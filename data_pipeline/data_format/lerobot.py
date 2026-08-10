from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from data_pipeline.data_format.alignment import pair_next_joint_targets
from data_pipeline.data_format.rosbag2 import (
    MessageRecord,
    Rosbag2Reader,
    decode_depth_image,
    decode_rgb_image,
    extract_numeric_vector,
)
from data_pipeline.data_format.schema import build_lerobot_features


@dataclass(frozen=True, slots=True)
class ImageTopicConfig:
    """Map one ROS image topic to an RGB or depth LeRobot feature."""

    name: str
    topic: str
    is_depth: bool = False


@dataclass(slots=True)
class PendingFrame:
    state_record: MessageRecord
    candidates: dict[str, MessageRecord]


@dataclass(frozen=True, slots=True)
class LeRobotConversionConfig:
    """Configuration for converting one raw rosbag2 episode to LeRobot."""

    input_bag: Path
    output_root: Path
    repo_id: str
    task: str
    fps: int
    image_topics: tuple[ImageTopicConfig, ...]
    state_topic: str = "/joint_states"
    state_field: str = "position"
    ee_pose_topic: str = "/rm_driver/udp_arm_position"
    force_topic: str = "/rm_driver/udp_six_zero_force"
    storage_id: str | None = None
    robot_type: str = "realman_eco63_6fb"
    tolerance_s: float = 0.05
    use_videos: bool = True
    joint_count: int = 6

    def validate(self) -> None:
        if self.fps <= 0:
            raise ValueError("fps 必须大于 0")
        if self.tolerance_s <= 0:
            raise ValueError("tolerance_s 必须大于 0")
        if self.joint_count != 6:
            raise ValueError("当前 RealMan ECO63 schema 固定为 6 个关节")
        if not self.repo_id or "/" not in self.repo_id:
            raise ValueError("repo_id 必须使用 namespace/name 格式")
        rgb_topics = [item for item in self.image_topics if not item.is_depth]
        if not rgb_topics:
            raise ValueError("必须至少配置一路 RGB topic")
        if len({item.name for item in self.image_topics}) != len(self.image_topics):
            raise ValueError("LeRobot 图像 feature name 必须唯一")
        if len({item.topic for item in self.image_topics}) != len(self.image_topics):
            raise ValueError("ROS 2 图像 topic 必须唯一")
        if any(not item.topic.startswith("/") for item in self.image_topics):
            raise ValueError("图像 topic 必须是绝对 ROS topic")
        if any(item.topic == self.state_topic for item in self.image_topics):
            raise ValueError("图像 topic 不允许复用 state topic")
        if not self.ee_pose_topic.startswith("/") or not self.force_topic.startswith("/"):
            raise ValueError("ee pose 和 force topic 必须是绝对 ROS topic")


class Rosbag2ToLeRobotConverter:
    """Align observations and derive next-joint-target actions for LeRobot."""

    def __init__(self, config: LeRobotConversionConfig) -> None:
        config.validate()
        self.config = config

    def _build_observation(self, pending: PendingFrame) -> dict[str, Any] | None:
        required_topics = {
            self.config.ee_pose_topic,
            self.config.force_topic,
            *(item.topic for item in self.config.image_topics),
        }
        if not required_topics.issubset(pending.candidates):
            return None
        state = extract_numeric_vector(
            pending.state_record.message,
            self.config.state_field,
        )
        if state.size < self.config.joint_count:
            raise ValueError(
                f"{self.config.state_topic}.{self.config.state_field} 少于 6 个关节"
            )
        state = state[: self.config.joint_count].astype(np.float32, copy=True)
        if not np.isfinite(state).all():
            raise ValueError("joint state 包含 NaN/Inf")
        observation: dict[str, Any] = {
            "observation.state": state,
            "observation.ee_pose": self._extract_ee_pose(
                pending.candidates[self.config.ee_pose_topic]
            ),
            "observation.force": self._extract_force(
                pending.candidates[self.config.force_topic]
            ),
            "task": self.config.task,
        }
        for image_config in self.config.image_topics:
            record = pending.candidates[image_config.topic]
            key = f"observation.images.{image_config.name}"
            observation[key] = (
                decode_depth_image(record) if image_config.is_depth else decode_rgb_image(record)
            )
        return observation

    @staticmethod
    def _extract_ee_pose(record: MessageRecord) -> np.ndarray:
        pose = record.message
        value = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ],
            dtype=np.float32,
        )
        if not np.isfinite(value).all():
            raise ValueError("末端位姿包含 NaN/Inf")
        return value

    @staticmethod
    def _extract_force(record: MessageRecord) -> np.ndarray:
        force = record.message
        value = np.asarray(
            [
                force.force_fx,
                force.force_fy,
                force.force_fz,
                force.force_mx,
                force.force_my,
                force.force_mz,
            ],
            dtype=np.float32,
        )
        if not np.isfinite(value).all():
            raise ValueError("六维力包含 NaN/Inf")
        return value

    def _aligned_observations(
        self,
        records: Iterator[MessageRecord],
    ) -> Iterator[dict[str, Any]]:
        tolerance_ns = round(self.config.tolerance_s * 1_000_000_000)
        interval_ns = round(1_000_000_000 / self.config.fps)
        candidate_topics = {
            self.config.ee_pose_topic,
            self.config.force_topic,
            *(item.topic for item in self.config.image_topics),
        }
        latest_by_topic: dict[str, MessageRecord] = {}
        pending_frames: list[PendingFrame] = []
        last_state_timestamp_ns: int | None = None

        for record in records:
            while (
                pending_frames
                and record.timestamp_ns
                > pending_frames[0].state_record.timestamp_ns + tolerance_ns
            ):
                observation = self._build_observation(pending_frames.pop(0))
                if observation is not None:
                    yield observation

            if record.topic == self.config.state_topic:
                if (
                    last_state_timestamp_ns is not None
                    and record.timestamp_ns - last_state_timestamp_ns < interval_ns
                ):
                    continue
                candidates = {
                    topic: candidate
                    for topic, candidate in latest_by_topic.items()
                    if record.timestamp_ns - candidate.timestamp_ns <= tolerance_ns
                }
                pending_frames.append(PendingFrame(record, candidates))
                last_state_timestamp_ns = record.timestamp_ns
                continue

            if record.topic not in candidate_topics:
                continue
            latest_by_topic[record.topic] = record
            for pending in pending_frames:
                difference_ns = abs(record.timestamp_ns - pending.state_record.timestamp_ns)
                if difference_ns > tolerance_ns:
                    continue
                current = pending.candidates.get(record.topic)
                if current is None or difference_ns < abs(
                    current.timestamp_ns - pending.state_record.timestamp_ns
                ):
                    pending.candidates[record.topic] = record

        for pending in pending_frames:
            observation = self._build_observation(pending)
            if observation is not None:
                yield observation

    def convert(self) -> Path:
        if self.config.output_root.exists():
            raise FileExistsError(f"输出路径已存在: {self.config.output_root}")
        topics = {
            self.config.state_topic,
            self.config.ee_pose_topic,
            self.config.force_topic,
            *(item.topic for item in self.config.image_topics),
        }
        records = Rosbag2Reader(
            self.config.input_bag,
            storage_id=self.config.storage_id,
        ).iter_records(topics)
        frames = pair_next_joint_targets(self._aligned_observations(records))
        try:
            first_frame = next(frames)
        except StopIteration as exc:
            raise ValueError(
                "至少需要两个完成对齐的 observation 才能构造 action"
            ) from exc

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise ImportError(
                "需要 Python 3.12 和 third_party/lerobot 的 dataset 依赖"
            ) from exc

        dataset = LeRobotDataset.create(
            repo_id=self.config.repo_id,
            fps=self.config.fps,
            features=build_lerobot_features(first_frame, use_videos=self.config.use_videos),
            root=self.config.output_root,
            robot_type=self.config.robot_type,
            use_videos=self.config.use_videos,
            image_writer_threads=4,
        )
        try:
            dataset.add_frame(first_frame)
            for frame in frames:
                dataset.add_frame(frame)
            dataset.save_episode()
        finally:
            dataset.finalize()
        return Path(dataset.root)
