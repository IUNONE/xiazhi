from __future__ import annotations

import json
import re
from copy import deepcopy
from dataclasses import dataclass, replace
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
    """Convert a raw session (or one rosbag2 episode) to one LeRobot dataset."""

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
    action_topic: str | None = None

    def validate(self) -> None:
        if self.action_topic is not None and not self.action_topic.startswith("/"):
            raise ValueError("action_topic 必须为绝对 ROS topic")
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
        self.action_topic = config.action_topic
        self.source_manifest = None
        manifest_path = config.input_bag / "session_manifest.json"
        if not manifest_path.is_file():
            manifest_path = config.input_bag.parent / "session_manifest.json"
        if manifest_path.is_file():
            self.source_manifest = json.loads(manifest_path.read_text())
            for episode in self.source_manifest.get("episodes", []):
                if Path(episode["output_path"]).name == config.input_bag.name and episode.get("status") != "completed":
                    raise ValueError(f"输入 episode 未成功结束，请先审查失败记录: {config.input_bag}")
            action = self.source_manifest.get("action", {})
            if action.get("source") == "leader_follower_submitted_joint_target":
                recorded_topic = action["topic"]
                if self.action_topic is not None and self.action_topic != recorded_topic:
                    raise ValueError("action_topic 与 session manifest 不一致")
                self.action_topic = recorded_topic


    def _build_observation(self, pending: PendingFrame) -> dict[str, Any] | None:
        required_topics = {
            self.config.ee_pose_topic,
            self.config.force_topic,
            *(item.topic for item in self.config.image_topics),
        }
        if self.action_topic is not None:
            required_topics.add(self.action_topic)
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
        if self.action_topic is not None:
            command = pending.candidates[self.action_topic]
            if command.type_name != "sensor_msgs/msg/JointState":
                raise TypeError("遥操 action 必须是 JointState")
            values = dict(zip(command.message.name, command.message.position))
            names = [f"joint_{i}" for i in range(1, 7)]
            if len(command.message.name) != len(command.message.position) or len(values) != 6 or set(values) != set(names):
                raise ValueError("遥操 action 关节名称/维度无效")
            target = np.asarray([values[n] for n in names], dtype=np.float32)
            if not np.isfinite(target).all():
                raise ValueError("遥操 action 包含非有限值")
            observation["action"] = target
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
        if self.action_topic is not None:
            candidate_topics.add(self.action_topic)
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
                    and topic != self.action_topic
                }
                pending_frames.append(PendingFrame(record, candidates))
                last_state_timestamp_ns = record.timestamp_ns
                continue

            if record.topic not in candidate_topics:
                continue
            latest_by_topic[record.topic] = record
            for pending in pending_frames:
                if record.topic == self.action_topic:
                    # First submitted command at/after state time; no nearest-past action.
                    delta = record.timestamp_ns - pending.state_record.timestamp_ns
                    if 0 <= delta <= tolerance_ns and record.topic not in pending.candidates:
                        pending.candidates[record.topic] = record
                    continue
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

    def _episode_paths(self) -> list[Path]:
        source = self.config.input_bag
        if source.is_file() or (source / "metadata.yaml").is_file():
            return [source]
        if not source.is_dir():
            raise FileNotFoundError(f"输入目录不存在: {source}")
        if not (source / "session_manifest.json").is_file() and any(source.glob("*.mcap")):
            return [source]
        episodes = sorted(
            (path for path in source.iterdir()
             if path.is_dir() and re.fullmatch(r"episode_\d+", path.name)),
            key=lambda path: (int(path.name.split("_")[1]), path.name),
        )
        if not episodes:
            raise ValueError(f"session 下没有 episode_<编号> 目录: {source}")
        # Resolve against the current session, even if it was moved after recording.
        if self.source_manifest is not None:
            found = {path.name for path in episodes}
            missing = [Path(item["output_path"]).name
                       for item in self.source_manifest.get("episodes", [])
                       if Path(item["output_path"]).name not in found]
            if missing:
                raise FileNotFoundError(f"session 缺少 manifest 中的 episode: {missing}")
        return episodes

    def _frames(self) -> Iterator[dict[str, Any]]:
        topics = {
            self.config.state_topic,
            self.config.ee_pose_topic,
            self.config.force_topic,
            *(item.topic for item in self.config.image_topics),
        }
        if self.action_topic is not None:
            topics.add(self.action_topic)
        records = Rosbag2Reader(
            self.config.input_bag,
            storage_id=self.config.storage_id,
        ).iter_records(topics)
        observations = self._aligned_observations(records)
        yield from (observations if self.action_topic is not None
                    else pair_next_joint_targets(observations))

    def convert(self) -> Path:
        if self.config.output_root.exists():
            raise FileExistsError(f"输出路径已存在: {self.config.output_root}")
        # Validate every episode's completion status before creating output.
        converters = [
            Rosbag2ToLeRobotConverter(replace(self.config, input_bag=path))
            for path in self._episode_paths()
        ]

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise ImportError(
                "需要 Python 3.12 和 third_party/lerobot 的 dataset 依赖"
            ) from exc

        dataset = None
        features = None
        episode_provenance = []
        try:
            for index, converter in enumerate(converters):
                path = converter.config.input_bag
                print(f"[{index + 1}/{len(converters)}] 转换 {path.name}", flush=True)
                frames = converter._frames()
                try:
                    first_frame = next(frames)
                    episode_features = build_lerobot_features(
                        first_frame, use_videos=self.config.use_videos,
                    )
                    if dataset is None:
                        features = episode_features
                        dataset = LeRobotDataset.create(
                            repo_id=self.config.repo_id,
                            fps=self.config.fps,
                            features=deepcopy(features),
                            root=self.config.output_root,
                            robot_type=self.config.robot_type,
                            use_videos=self.config.use_videos,
                            image_writer_threads=4,
                        )
                    elif episode_features != features:
                        raise ValueError(f"episode 的图像尺寸或 feature schema 不一致: {path}")
                    dataset.add_frame(first_frame)
                    frame_count = 1
                    for frame in frames:
                        dataset.add_frame(frame)
                        frame_count += 1
                    dataset.save_episode()
                    episode_provenance.append({
                        "episode_index": index,
                        "source_episode": path.name,
                        "source_path": str(path),
                        "frame_count": frame_count,
                    })
                    print(f"  已保存 episode {index}: {frame_count} 帧", flush=True)
                except StopIteration as exc:
                    raise ValueError(
                        f"{path}: 没有完整的 observation/action；"
                        "拖动需要两帧，遥操需要有效命令流"
                    ) from exc
                finally:
                    frames.close()
        finally:
            if dataset is not None:
                dataset.finalize()
        provenance = {
            "source": "leader_follower_submitted_joint_target" if self.action_topic else "derived_next_joint_target",
            "unit": "rad", "topic": self.action_topic,
            "alignment": "first_command_at_or_after_observation" if self.action_topic else "next_observation",
            "tolerance_s": self.config.tolerance_s,
            "source_session": self.source_manifest,
            "episodes": episode_provenance,
        }
        with (Path(dataset.root) / "action_provenance.json").open("x", encoding="utf-8") as stream:
            json.dump(provenance, stream, ensure_ascii=False, indent=2)
        return Path(dataset.root)
