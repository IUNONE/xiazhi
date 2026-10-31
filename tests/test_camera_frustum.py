from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from data_pipeline.data_rerun.camera import (
    CameraIntrinsics, lerobot_intrinsics, log_wrist_camera, read_mcap_intrinsics,
)


class CameraFallbackTests(unittest.TestCase):
    def test_missing_source_uses_explicit_illustrative_intrinsics(self):
        with tempfile.TemporaryDirectory() as temp:
            camera = lerobot_intrinsics(Path(temp), 0, 640, 480)
        self.assertIn("示意", camera.source)
        self.assertAlmostEqual(camera.matrix[0], 320 * np.sqrt(3))

    def test_resize_preserves_projection(self):
        camera = CameraIntrinsics(640, 480, (500, 0, 320, 0, 510, 240, 0, 0, 1), "test")
        resized = camera.resized(320, 240)
        np.testing.assert_allclose(resized.matrix, (250, 0, 160, 0, 255, 120, 0, 0, 1))


@unittest.skipUnless(importlib.util.find_spec("mcap_ros2"), "requires mcap-ros2-support")
class McapCalibrationTests(unittest.TestCase):
    def test_real_cdr_camera_info_and_lerobot_provenance(self):
        from mcap_ros2.writer import Writer

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "episode_000001"
            source.mkdir()
            path = source / "data.mcap"
            with Writer(str(path)) as writer:
                schema = writer.register_msgdef("sensor_msgs/msg/CameraInfo", "uint32 width\nuint32 height\nfloat64[9] k\n")
                writer.write_message("/camera_01/color/camera_info", schema, {
                    "width": 640, "height": 480, "k": [500, 0, 320, 0, 500, 240, 0, 0, 1],
                }, log_time=10)
            camera = read_mcap_intrinsics((path,))
            self.assertEqual(camera.source, "MCAP CameraInfo")
            self.assertEqual(camera.matrix[0], 500)
            (root / "action_provenance.json").write_text(json.dumps({"episodes": [
                {"episode_index": 1, "source_path": str(source)},
            ]}))
            camera = lerobot_intrinsics(root, 1, 320, 240)
            self.assertEqual(camera.matrix[0], 250)
            self.assertEqual(camera.source, "MCAP CameraInfo")


@unittest.skipUnless(importlib.util.find_spec("rerun"), "requires rerun-sdk 0.33")
class CameraRecordingTests(unittest.TestCase):
    def test_rgb_is_bound_to_named_pinhole_plane_at_eef(self):
        import rerun as rr

        with rr.RecordingStream("camera-test") as stream:
            recording = Mock(wraps=stream)
            log_wrist_camera(recording, rr, "/camera_01/color/image_raw",
                             CameraIntrinsics.illustrative(640, 480), "Link6")
            calls = {call.args[0]: call.args[1] for call in recording.log.call_args_list}
            mount = calls["/camera_frustums/camera_01/mount"]
            self.assertEqual(mount.parent_frame.as_arrow_array().to_pylist(), ["Link6"])
            self.assertEqual(mount.translation.as_arrow_array().to_pylist(), [[0, 0, 0]])
            pinhole = calls["/camera_frustums/camera_01/pinhole"]
            self.assertEqual(pinhole.parent_frame.as_arrow_array().to_pylist(),
                             mount.child_frame.as_arrow_array().to_pylist())
            image = calls["/camera_01/color/image_raw"]
            self.assertEqual(image.frame.as_arrow_array().to_pylist(),
                             pinhole.child_frame.as_arrow_array().to_pylist())

    def test_lerobot_animates_robot_on_same_timelines_without_decoding_images(self):
        import rerun as rr
        from data_pipeline.data_rerun.lerobot_scene import build_blueprint, log_scene

        # A small real URDF exercises Rerun's URDF parser and joint transforms.
        urdf = '<robot name="test"><link name="base_root"/>'
        for i in range(1, 7):
            parent = "base_root" if i == 1 else f"Link{i - 1}"
            urdf += f'''<link name="Link{i}"><visual><geometry><box size="0.1 0.02 0.02"/></geometry></visual></link>
                <joint name="joint_{i}" type="revolute"><parent link="{parent}"/><child link="Link{i}"/>
                <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
                <limit lower="-4" upper="4" effort="10" velocity="10"/></joint>'''
        urdf += '</robot>'
        rows = [{"index": 100 + i, "episode_index": 1, "timestamp": i / 10,
                 "observation.state": [i / 10] * 6, "observation.force": [i] * 6}
                for i in range(2)]
        table = Mock()
        table.column_names = list(rows[0])
        table.select_columns.return_value.with_format.return_value = rows
        with tempfile.TemporaryDirectory() as temp:
            dataset = SimpleNamespace(root=Path(temp), hf_dataset=table,
                                      meta=SimpleNamespace(camera_keys=["observation.images.camera_01"]),
                                      features={
                "observation.state": {"names": [f"joint_{i}" for i in range(1, 7)]},
                "observation.force": {},
                "observation.images.camera_01": {"shape": [48, 64, 3], "names": ["height", "width", "channels"]},
            })
            with rr.RecordingStream("lerobot-scene-test") as stream:
                stream.save(Path(temp) / "scene.rrd")
                stream.send_blueprint(build_blueprint(dataset))
                recording = Mock(wraps=stream)
                with patch("data_pipeline.data_rerun.lerobot_scene.build_robot_urdf", return_value=urdf.encode()):
                    log_scene(dataset, recording, Path("robot.xacro"), "Link6", 1)
                joint_logs = [call.args[1] for call in recording.log.call_args_list
                              if call.args[0] == "/robot_joints/joint_6"]
                self.assertEqual(len(joint_logs), 2)
                self.assertNotEqual(joint_logs[0].quaternion.as_arrow_array().to_pylist(),
                                    joint_logs[1].quaternion.as_arrow_array().to_pylist())
                times = [(c.args[0], c.kwargs) for c in recording.set_time.call_args_list]
                self.assertEqual(times, [("frame_index", {"sequence": 0}), ("timestamp", {"timestamp": 0.0}),
                                         ("frame_index", {"sequence": 1}), ("timestamp", {"timestamp": 0.1})])
                self.assertNotIn("observation.images.camera_01", table.select_columns.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
