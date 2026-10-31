"""Add the robot and wrist-camera frustum to the official LeRobot image logs."""
from __future__ import annotations

from pathlib import Path
import tempfile

import numpy as np

from data_pipeline.data_rerun.camera import lerobot_intrinsics, log_wrist_camera, robot_view
from data_pipeline.data_rerun.robot_model import build_robot_urdf


def build_blueprint(dataset):
    import rerun.blueprint as rrb

    cameras = [rrb.Spatial2DView(origin=key, name=key.removeprefix("observation.images."))
               for key in dataset.meta.camera_keys]
    signals = [rrb.TimeSeriesView(origin=origin, name=name) for origin, name, key in (
        ("state", "Joints", "observation.state"),
        ("action", "Action", "action"),
        ("observation.force", "EEF Force", "observation.force"),
        ("observation.ee_pose", "EEF Pose", "observation.ee_pose"),
    ) if key in dataset.features]
    scene = robot_view(rrb, ["/observation.images.camera_01"])
    return rrb.Blueprint(rrb.Vertical(
        rrb.TextDocumentView(origin="task", name="Instruction"),
        rrb.Horizontal(scene, rrb.Grid(*cameras), column_shares=[3, 1]) if cameras else scene,
        rrb.Horizontal(*signals), row_shares=[1, 7, 2],
    ))


def log_scene(dataset, recording, robot_xacro: Path, eef_link: str, episode_index: int) -> None:
    import rerun as rr
    from rerun.urdf import UrdfTree

    urdf = build_robot_urdf(robot_xacro)
    recording.reset_time()
    with tempfile.TemporaryDirectory(prefix="lerobot-model-") as temp:
        path = Path(temp) / "robot.urdf"
        path.write_bytes(urdf)
        tree = UrdfTree.from_file_path(path, entity_path_prefix="robot_model")
        tree.log_urdf_to_recording(recording)
    if tree.get_link_by_name(eef_link) is None:
        raise ValueError(f"URDF 中没有末端 link: {eef_link}")
    state_feature = dataset.features.get("observation.state")
    if state_feature is None:
        raise ValueError("机械臂可视化需要 observation.state；可使用 --no-robot-model")
    names = state_feature.get("names") or [f"joint_{i}" for i in range(1, 7)]
    joints = [tree.get_joint_by_name(name) for name in names]
    if any(joint is None for joint in joints):
        raise ValueError(f"observation.state 关节名称与 URDF 不一致: {names}")

    image_key = "observation.images.camera_01"
    if image_key in dataset.meta.camera_keys:
        feature = dataset.features[image_key]
        dimensions = dict(zip(feature.get("names") or [], feature["shape"]))
        if "height" in dimensions and "width" in dimensions:
            width, height = dimensions["width"], dimensions["height"]
        elif feature["shape"][-1] in (1, 3, 4):
            height, width = feature["shape"][:2]
        else:
            height, width = feature["shape"][-2:]
        intrinsics = lerobot_intrinsics(Path(dataset.root), episode_index, width, height)
        log_wrist_camera(recording, rr, image_key, intrinsics, eef_link)

    # Read only numeric columns: images were already decoded by LeRobot's exporter.
    columns = [key for key in ("index", "episode_index", "timestamp", "observation.state",
                              "observation.force", "observation.ee_pose")
               if key in dataset.hf_dataset.column_names]
    rows = dataset.hf_dataset.select_columns(columns).with_format(None)
    first_index = None
    for row in rows:
        if row["episode_index"] != episode_index:
            continue
        if first_index is None:
            first_index = row["index"]
        recording.set_time("frame_index", sequence=int(row["index"] - first_index))
        recording.set_time("timestamp", timestamp=float(row["timestamp"]))
        values = np.asarray(row["observation.state"]).reshape(-1)
        if len(values) != len(joints) or not np.isfinite(values).all():
            raise ValueError("observation.state 关节维度无效或包含 NaN/Inf")
        for joint, value in zip(joints, values):
            recording.log(f"/robot_joints/{joint.name}", joint.compute_transform(float(value), clamp=False))
        for key in ("observation.force", "observation.ee_pose"):
            if key in row:
                recording.log(key, rr.Scalars(row[key]))
    recording.reset_time()
    recording.flush()
