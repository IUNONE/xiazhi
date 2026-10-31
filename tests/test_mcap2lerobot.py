from __future__ import annotations

import json
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np

from data_pipeline.data_format import ImageTopicConfig, LeRobotConversionConfig, Rosbag2ToLeRobotConverter
from data_pipeline.data_format.rosbag2 import MessageRecord
from scripts.convert_mcap2lerobot import main


class RecordingDataset:
    """Capture calls at the external LeRobot writer boundary."""

    def __init__(self, **kwargs):
        self.options = kwargs
        self.root = kwargs["root"]
        self.root.mkdir()
        self.episodes = []
        self.pending = []
        self.finalized = False

    def add_frame(self, frame):
        self.pending.append(frame)

    def save_episode(self):
        self.episodes.append(self.pending)
        self.pending = []

    def finalize(self):
        self.finalized = True


class SessionConversionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.session = self.root / "session"
        self.session.mkdir()
        (self.session / "teleoperation").mkdir()
        self.manifest = {
            "task": "pick", "cameras": [{"name": "camera_01", "enable_depth": False}],
            "episodes": [],
        }
        self.config = LeRobotConversionConfig(
            input_bag=self.session, output_root=self.root / "out", repo_id="local/test",
            task="pick", fps=10, storage_id="mcap",
            image_topics=(ImageTopicConfig("camera_01", "/camera_01/color/image_raw"),),
        )
        self.records = {}
        self.datasets = []
        module = ModuleType("lerobot.datasets.lerobot_dataset")
        module.LeRobotDataset = SimpleNamespace(create=self.create_dataset)
        self.addCleanup(patch.stopall)
        patch.dict(sys.modules, {"lerobot.datasets.lerobot_dataset": module}).start()
        reader = patch("data_pipeline.data_format.lerobot.Rosbag2Reader").start()
        reader.side_effect = lambda path, storage_id: SimpleNamespace(
            iter_records=lambda topics: iter(self.records[path.name]),
        )

    def create_dataset(self, **kwargs):
        dataset = RecordingDataset(**kwargs)
        self.datasets.append(dataset)
        return dataset

    def write_manifest(self):
        (self.session / "session_manifest.json").write_text(json.dumps(self.manifest))

    def add_episode(self, name, offset=0, status="completed", command=False, width=2):
        path = self.session / name
        path.mkdir()
        (path / "metadata.yaml").write_text("storage_identifier: mcap\n")
        self.manifest["episodes"].append({
            "output_path": f"/old/recording/location/{name}", "status": status,
        })
        self.write_manifest()
        records = []
        for index in range(5):
            timestamp = index * 100_000_000
            def record(topic, message, type_name="test/msg/Message"):
                records.append(MessageRecord(topic, type_name, timestamp, message))
            record(self.config.ee_pose_topic, SimpleNamespace(
                position=SimpleNamespace(x=0, y=0, z=0),
                orientation=SimpleNamespace(w=1, x=0, y=0, z=0),
            ))
            record(self.config.force_topic, SimpleNamespace(**{
                f"force_{axis}": 0 for axis in ("fx", "fy", "fz", "mx", "my", "mz")
            }))
            record(self.config.image_topics[0].topic, SimpleNamespace(
                encoding="rgb8", height=2, width=width, step=width * 3, data=bytes(width * 6),
            ), "sensor_msgs/msg/Image")
            record(self.config.state_topic, SimpleNamespace(position=[offset + index] * 6))
            if command:
                record("/command", SimpleNamespace(
                    name=[f"joint_{i}" for i in range(1, 7)], position=[offset + index + 100] * 6,
                ), "sensor_msgs/msg/JointState")
        self.records[name] = records
        return path

    def test_all_episodes_numeric_order_and_actions_do_not_cross_boundaries(self):
        self.add_episode("episode_10", offset=10)
        self.add_episode("episode_000002", offset=2)
        output = Rosbag2ToLeRobotConverter(self.config).convert()
        self.assertEqual(len(self.datasets), 1)
        dataset = self.datasets[0]
        self.assertTrue(dataset.finalized)
        self.assertEqual([len(episode) for episode in dataset.episodes], [4, 4])
        np.testing.assert_array_equal(dataset.episodes[0][-1]["action"], [6] * 6)
        np.testing.assert_array_equal(dataset.episodes[1][0]["observation.state"], [10] * 6)
        provenance = json.loads((output / "action_provenance.json").read_text())
        self.assertEqual([e["source_episode"] for e in provenance["episodes"]],
                         ["episode_000002", "episode_10"])
        self.assertEqual([e["episode_index"] for e in provenance["episodes"]], [0, 1])

    def test_fps_sampling_resets_for_each_episode(self):
        self.add_episode("episode_000000")
        self.add_episode("episode_000001", offset=10)
        Rosbag2ToLeRobotConverter(replace(self.config, fps=5)).convert()
        dataset = self.datasets[0]
        self.assertEqual(dataset.options["fps"], 5)
        self.assertEqual([len(episode) for episode in dataset.episodes], [2, 2])
        np.testing.assert_array_equal(dataset.episodes[0][0]["action"], [2] * 6)

    def test_three_argument_cli_infers_manifest_and_preserves_teleop_actions(self):
        self.manifest["action"] = {"source": "leader_follower_submitted_joint_target", "topic": "/command"}
        self.add_episode("episode_000000", command=True)
        self.add_episode("episode_000001", command=True)
        with patch.object(sys, "argv", ["convert_mcap2lerobot.py", "--mcap_dir", str(self.session),
                                       "--output_dir", str(self.config.output_root), "--fps", "10"]):
            main()
        dataset = self.datasets[0]
        self.assertEqual(dataset.options["repo_id"], "local/out")
        self.assertEqual([len(episode) for episode in dataset.episodes], [5, 5])
        self.assertEqual(dataset.episodes[0][0]["task"], "pick")
        np.testing.assert_array_equal(dataset.episodes[0][0]["action"], [100] * 6)

    def test_failed_episode_rejected_before_output_is_created(self):
        self.add_episode("episode_000000")
        self.add_episode("episode_000001", status="failed")
        with self.assertRaisesRegex(ValueError, "episode_000001"):
            Rosbag2ToLeRobotConverter(self.config).convert()
        self.assertFalse(self.config.output_root.exists())

    def test_cli_infers_depth_and_allows_explicit_camera_mapping(self):
        self.manifest["cameras"] = [
            {"name": "camera_01", "enable_depth": True},
            {"name": "camera_02", "enable_depth": False},
        ]
        self.write_manifest()
        argv = ["convert_mcap2lerobot.py", "--mcap_dir", str(self.session),
                "--output_dir", str(self.config.output_root), "--fps", "30"]
        with patch("scripts.convert_mcap2lerobot.Rosbag2ToLeRobotConverter") as converter:
            with patch.object(sys, "argv", argv):
                main()
            config = converter.call_args.args[0]
            self.assertEqual([(item.name, item.is_depth) for item in config.image_topics],
                             [("camera_01", False), ("camera_01_depth", True), ("camera_02", False)])
            with patch.object(sys, "argv", argv + ["--image-topic", "front=/custom/rgb"]):
                main()
            config = converter.call_args.args[0]
            self.assertEqual(config.image_topics, (ImageTopicConfig("front", "/custom/rgb"),))

    def test_missing_manifest_episode_is_not_silently_skipped(self):
        self.add_episode("episode_000000")
        self.manifest["episodes"].append({"output_path": "/old/episode_000001", "status": "completed"})
        self.write_manifest()
        with self.assertRaisesRegex(FileNotFoundError, "episode_000001"):
            Rosbag2ToLeRobotConverter(self.config).convert()

    def test_empty_session_rejected(self):
        with self.assertRaisesRegex(ValueError, "没有 episode"):
            Rosbag2ToLeRobotConverter(self.config).convert()

    def test_empty_later_episode_reports_source_and_finalizes(self):
        self.add_episode("episode_000000")
        self.add_episode("episode_000001")
        self.records["episode_000001"] = []
        with self.assertRaisesRegex(ValueError, "episode_000001.*没有完整"):
            Rosbag2ToLeRobotConverter(self.config).convert()
        self.assertTrue(self.datasets[0].finalized)
        self.assertEqual(len(self.datasets[0].episodes), 1)
        self.assertFalse((self.config.output_root / "action_provenance.json").exists())

    def test_incompatible_image_shapes_rejected(self):
        self.add_episode("episode_000000")
        self.add_episode("episode_000001", width=3)
        with self.assertRaisesRegex(ValueError, "schema 不一致"):
            Rosbag2ToLeRobotConverter(self.config).convert()
        self.assertTrue(self.datasets[0].finalized)

    def test_existing_output_is_never_overwritten(self):
        self.add_episode("episode_000000")
        self.config.output_root.mkdir()
        with self.assertRaises(FileExistsError):
            Rosbag2ToLeRobotConverter(self.config).convert()
        self.assertEqual(self.datasets, [])

    def test_single_episode_remains_supported(self):
        path = self.add_episode("episode_000000")
        Rosbag2ToLeRobotConverter(replace(self.config, input_bag=path)).convert()
        self.assertEqual(len(self.datasets[0].episodes), 1)


if __name__ == "__main__":
    unittest.main()
