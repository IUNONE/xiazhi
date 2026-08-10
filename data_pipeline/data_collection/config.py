from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


ROBOT_TOPICS = (
    "/joint_states",
    "/rm_driver/udp_arm_position",
    "/rm_driver/udp_six_force",
    "/rm_driver/udp_six_zero_force",
    "/rm_driver/udp_joint_error_code",
    "/rm_driver/udp_rm_err",
    "/tf",
    "/tf_static",
)

CAMERA_TOPIC_SUFFIXES = (
    "color/image_raw",
    "color/camera_info",
    "depth/image_raw",
    "depth/camera_info",
)


@dataclass(frozen=True, slots=True)
class RealManConfig:
    """RealMan ECO63 direct-SDK connection and drag-teach settings."""

    ip: str = "192.168.1.18"
    port: int = 8080
    drag_mode: int = 3
    singular_wall: bool = True
    home_return_speed: int = 10
    home_tolerance_deg: float = 1.0

    def validate(self) -> None:
        if not self.ip:
            raise ValueError("robot ip 不允许为空")
        if not 1 <= self.port <= 65535:
            raise ValueError("robot port 必须在 1 到 65535")
        if self.drag_mode not in {0, 1, 2, 3}:
            raise ValueError("drag_mode 必须是 0, 1, 2 或 3")
        if not 1 <= self.home_return_speed <= 100:
            raise ValueError("home_return_speed 必须在 1 到 100")
        if self.home_tolerance_deg <= 0:
            raise ValueError("home_tolerance_deg 必须大于 0")


@dataclass(frozen=True, slots=True)
class CameraConfig:
    """Bind one Gemini 335 to a stable name by its hardware serial number."""

    name: str
    serial_number: str
    fps: int = 30
    width: int = 640
    height: int = 480
    enable_depth: bool = True
    warmup_s: float = 1.0

    def validate(self) -> None:
        if not self.name or Path(self.name).name != self.name:
            raise ValueError("camera name 必须是单层非空名称")
        if not self.serial_number:
            raise ValueError("camera serial_number 不允许为空")
        if self.fps <= 0 or self.width <= 0 or self.height <= 0:
            raise ValueError("camera fps, width 和 height 必须大于 0")
        if self.warmup_s <= 0:
            raise ValueError("camera warmup_s 必须大于 0")

    def topics(self) -> tuple[str, ...]:
        namespace = self.name.strip("/")
        suffixes = CAMERA_TOPIC_SUFFIXES if self.enable_depth else CAMERA_TOPIC_SUFFIXES[:2]
        return tuple(f"/{namespace}/{suffix}" for suffix in suffixes)

    def sample_topics(self) -> tuple[str, ...]:
        namespace = self.name.strip("/")
        topics = [f"/{namespace}/color/image_raw"]
        if self.enable_depth:
            topics.append(f"/{namespace}/depth/image_raw")
        return tuple(topics)


@dataclass(frozen=True, slots=True)
class CollectionConfig:
    """Shared configuration for one multi-episode kinesthetic-teaching session."""

    output_root: Path
    task: str
    scene: str
    session_id: str
    cameras: tuple[CameraConfig, ...]
    robot: RealManConfig = field(default_factory=RealManConfig)
    extra_topics: tuple[str, ...] = ()
    storage_id: str = "mcap"
    startup_timeout_s: float = 45.0
    sample_timeout_s: float = 5.0
    fps: int = 30
    allow_sdk_driver_coexistence: bool = False

    def validate(self) -> None:
        if len(self.cameras) != 3:
            raise ValueError("cameras 必须包含三台 Gemini 335")
        for camera in self.cameras:
            camera.validate()
        if len({camera.name for camera in self.cameras}) != 3:
            raise ValueError("camera name 必须唯一")
        if len({camera.serial_number for camera in self.cameras}) != 3:
            raise ValueError("camera serial_number 必须唯一")
        if self.storage_id not in {"sqlite3", "mcap"}:
            raise ValueError("storage_id 仅支持 sqlite3 或 mcap")
        if self.startup_timeout_s <= 0 or self.sample_timeout_s <= 0:
            raise ValueError("startup_timeout_s 和 sample_timeout_s 必须大于 0")
        if self.fps <= 0:
            raise ValueError("fps 必须大于 0")
        self.robot.validate()
        for value_name, value in (
            ("task", self.task),
            ("scene", self.scene),
            ("session_id", self.session_id),
        ):
            if not value or Path(value).name != value:
                raise ValueError(f"{value_name} 必须是单层非空名称")
        for topic in self.extra_topics:
            if not topic.startswith("/"):
                raise ValueError(f"extra topic 必须是绝对 ROS topic: {topic}")

    def output_path(self) -> Path:
        return self.output_root / self.task / self.scene / self.session_id

    def episode_output_path(self, episode_index: int) -> Path:
        if episode_index < 0:
            raise ValueError("episode_index 不允许为负数")
        return self.output_path() / f"episode_{episode_index:06d}"

    def topics(self) -> tuple[str, ...]:
        camera_topics = tuple(topic for camera in self.cameras for topic in camera.topics())
        return tuple(dict.fromkeys(ROBOT_TOPICS + camera_topics + self.extra_topics))

    def fresh_sample_topics(self) -> tuple[str, ...]:
        camera_topics = tuple(topic for camera in self.cameras for topic in camera.sample_topics())
        return ("/joint_states", "/rm_driver/udp_six_zero_force", *camera_topics)
