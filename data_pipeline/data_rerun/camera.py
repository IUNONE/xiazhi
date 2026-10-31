"""Shared wrist-camera projection for MCAP and LeRobot visualizations."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    matrix: tuple[float, ...]
    source: str

    @classmethod
    def illustrative(cls, width: int, height: int) -> CameraIntrinsics:
        # Existing LeRobot exports contain pixels but no CameraInfo calibration.
        focal = width / (2 * math.tan(math.radians(60) / 2))
        return cls(width, height, (focal, 0, width / 2, 0, focal, height / 2, 0, 0, 1),
                   "示意内参（水平视场角 60°，非标定值）")

    def resized(self, width: int, height: int) -> CameraIntrinsics:
        matrix = np.asarray(self.matrix, dtype=float).reshape(3, 3).copy()
        matrix[0] *= width / self.width
        matrix[1] *= height / self.height
        return CameraIntrinsics(width, height, tuple(matrix.ravel()), self.source)


def read_mcap_intrinsics(paths: tuple[Path, ...], camera: str = "camera_01") -> CameraIntrinsics | None:
    """Read the first usable RGB calibration; fall back to recorded image dimensions."""
    from mcap.reader import make_reader
    from data_pipeline.data_format.rosbag2 import Rosbag2Reader

    info_topic = f"/{camera}/color/camera_info"
    image_topic = f"/{camera}/color/image_raw"
    fallback = None
    for path in paths:
        with path.open("rb") as stream:
            reader = make_reader(stream)
            summary = reader.get_summary()
            available = ({channel.topic for channel in summary.channels.values()} if summary is not None
                         else {channel.topic for _, channel, _ in reader.iter_messages()})
        for topic in (info_topic, image_topic):
            if topic not in available:
                continue
            records = Rosbag2Reader(path, storage_id="mcap").iter_records({topic})
            try:
                for record in records:
                    msg = record.message
                    if msg.width <= 0 or msg.height <= 0:
                        continue
                    if topic == info_topic:
                        matrix = tuple(float(x) for x in msg.k)
                        if len(matrix) == 9 and np.isfinite(matrix).all() and matrix[0] > 0 and matrix[4] > 0:
                            return CameraIntrinsics(msg.width, msg.height, matrix, "MCAP CameraInfo")
                    else:
                        fallback = CameraIntrinsics.illustrative(msg.width, msg.height)
                        break
            finally:
                records.close()
    return fallback


def lerobot_intrinsics(root: Path, episode_index: int, width: int, height: int) -> CameraIntrinsics:
    """Reuse calibration from the original episode if it is still available."""
    provenance_path = root / "action_provenance.json"
    if provenance_path.is_file():
        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        episodes = provenance.get("episodes", [])
        if not episodes:
            episodes = (provenance.get("source_session") or {}).get("episodes", [])
        for index, episode in enumerate(episodes):
            if episode.get("episode_index", index) != episode_index:
                continue
            source = episode.get("source_path") or episode.get("output_path")
            if source:
                source = Path(source)
                paths = tuple(sorted(source.glob("*.mcap"))) if source.is_dir() else (source,) if source.is_file() else ()
                if paths:
                    intrinsics = read_mcap_intrinsics(paths)
                    if intrinsics is not None:
                        return intrinsics.resized(width, height)
    return CameraIntrinsics.illustrative(width, height)


def log_wrist_camera(recording, rr, image_path: str, intrinsics: CameraIntrinsics, eef_link: str) -> None:
    """Attach a display-only RGB camera to the EEF, with no calibrated offset."""
    optical_frame = "xiazhi/wrist_camera/optical"
    image_frame = "xiazhi/wrist_camera/image_plane"
    recording.log("/camera_frustums/camera_01/mount", rr.Transform3D(
        translation=[0, 0, 0], quaternion=rr.Quaternion(xyzw=[0, 0, 0, 1]),
        parent_frame=eef_link, child_frame=optical_frame,
    ), static=True)
    recording.log("/camera_frustums/camera_01/pinhole", rr.Pinhole(
        image_from_camera=np.asarray(intrinsics.matrix).reshape(3, 3),
        resolution=[intrinsics.width, intrinsics.height], camera_xyz=rr.ViewCoordinates.RDF,
        parent_frame=optical_frame, child_frame=image_frame, image_plane_distance=0.2,
    ), static=True)
    # Reuse the image already logged by the official importer/exporter, without
    # duplicating video pixels. The named image plane makes it a frustum texture.
    recording.log(image_path, rr.CoordinateFrame(image_frame), static=True)
    print(f"camera_01 视锥: 默认安装于 {eef_link} 原点，光轴沿末端 +Z；{intrinsics.source}")


def robot_view(rrb, image_paths: list[str]):
    return rrb.Spatial3DView(
        name="ECO63 Robot + wrist camera", origin="/",
        contents=["/robot_model/**", "/tf", "/tf_static", "/robot_joints/**",
                  "/camera_frustums/**", *image_paths],
        background=[18, 22, 30],
        spatial_information=rrb.SpatialInformation(target_frame="base_root", show_axes=True,
                                                   show_bounding_box=False),
        eye_controls=rrb.EyeControls3D(position=(1.4, -1.4, 1.2), look_target=(0, 0, 0.45),
                                      eye_up=(0, 0, 1)),
    )
