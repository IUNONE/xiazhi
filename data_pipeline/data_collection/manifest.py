from __future__ import annotations

import json
import platform
import subprocess
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from data_pipeline.data_collection.config import CollectionConfig


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _git_commit(path: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


@dataclass(slots=True)
class SessionManifest:
    """Provenance and per-episode completion state for one collection session."""

    backend: str
    task: str
    scene: str
    session_id: str
    robot: dict[str, Any]
    cameras: list[dict[str, Any]]
    output_path: str
    started_at: str = field(default_factory=_utc_now)
    finished_at: str | None = None
    status: str = "starting"
    stop_reason: str | None = None
    error: str | None = None
    episodes: list[dict[str, Any]] = field(default_factory=list)
    teaching_attempts: list[dict[str, Any]] = field(default_factory=list)
    action: dict[str, Any] = field(default_factory=dict)
    collection: dict[str, Any] = field(default_factory=dict)
    software: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_config(cls, config: CollectionConfig, backend: str) -> SessionManifest:
        """根据采集配置创建 session manifest。"""
        repository_root = Path(__file__).resolve().parents[2]
        software = {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "project_commit": _git_commit(repository_root),
            "lerobot_commit": _git_commit(repository_root / "third_party" / "lerobot"),
            "ros2_rm_robot_commit": _git_commit(
                repository_root / "third_party" / "ros2_rm_robot"
            ),
            "orbbec_ros2_commit": _git_commit(
                repository_root / "third_party" / "OrbbecSDK_ROS2"
            ),
        }
        action = {
            "source": (
                "derived_next_joint_target"
                if backend == "lerobot"
                else "derived_next_joint_target_during_lerobot_conversion"
            ),
            "unit": "rad",
            "horizon_frames": 1,
            "last_observation": "dropped",
        }
        return cls(
            backend=backend,
            task=config.task,
            scene=config.scene,
            session_id=config.session_id,
            robot={
                "model": "RealMan ECO63 6FB",
                "ip": config.robot.ip,
                "port": config.robot.port,
                "init_joints_deg": config.robot.init_joints_deg,
                "drag_mode": config.robot.drag_mode,
                "force_drag_mode": config.robot.force_drag_mode,
                "singular_wall": config.robot.singular_wall,
            },
            cameras=[asdict(camera) for camera in config.cameras],
            output_path=str(config.output_path()),
            action=action,
            collection={
                "capture_mode": config.capture_mode,
                "replay_pre_roll_s": config.replay_pre_roll_s,
                "replay_post_roll_s": config.replay_post_roll_s,
            },
            software=software,
        )

    def start_teaching_attempt(
        self,
        attempt_index: int,
        trajectory_path: Path,
    ) -> dict[str, Any]:
        """登记一次不录制 rosbag2 的拖动示教尝试。"""
        attempt = {
            "attempt_index": attempt_index,
            "trajectory_path": str(trajectory_path),
            "started_at": _utc_now(),
            "teaching_finished_at": None,
            "finished_at": None,
            "status": "teaching",
            "stop_reason": None,
            "trajectory_point_count": None,
            "episode_index": None,
            "error": None,
        }
        self.teaching_attempts.append(attempt)
        self.status = "teaching"
        return attempt

    def mark_teaching_ready(
        self,
        attempt: dict[str, Any],
        trajectory_point_count: int,
    ) -> None:
        """记录示教结束和控制器轨迹保存结果。"""
        attempt["teaching_finished_at"] = _utc_now()
        attempt["status"] = "ready_for_replay"
        attempt["trajectory_point_count"] = trajectory_point_count
        self.status = "waiting_for_origin"

    def finish_teaching_attempt(
        self,
        attempt: dict[str, Any],
        *,
        status: str,
        stop_reason: str,
        episode_index: int | None = None,
        error: BaseException | None = None,
    ) -> None:
        """结束一次示教尝试并记录关联 episode。"""
        attempt["finished_at"] = _utc_now()
        attempt["status"] = status
        attempt["stop_reason"] = stop_reason
        attempt["episode_index"] = episode_index
        attempt["error"] = None if error is None else f"{type(error).__name__}: {error}"
        self.status = "ready"

    def start_episode(self, episode_index: int, output_path: Path) -> dict[str, Any]:
        episode = {
            "episode_index": episode_index,
            "output_path": str(output_path),
            "started_at": _utc_now(),
            "finished_at": None,
            "status": "recording",
            "stop_reason": None,
            "frame_count": None,
            "error": None,
        }
        self.episodes.append(episode)
        self.status = "recording"
        return episode

    def finish_episode(
        self,
        episode: dict[str, Any],
        *,
        status: str,
        stop_reason: str,
        frame_count: int | None = None,
        error: BaseException | None = None,
    ) -> None:
        episode["finished_at"] = _utc_now()
        episode["status"] = status
        episode["stop_reason"] = stop_reason
        episode["frame_count"] = frame_count
        episode["error"] = None if error is None else f"{type(error).__name__}: {error}"
        self.status = "ready"

    def finish_session(
        self,
        *,
        status: str,
        stop_reason: str,
        error: BaseException | None = None,
    ) -> None:
        self.finished_at = _utc_now()
        self.status = status
        self.stop_reason = stop_reason
        self.error = None if error is None else f"{type(error).__name__}: {error}"

    def write(self, output_dir: Path) -> Path:
        output_dir.mkdir(parents=True, exist_ok=True)
        destination = output_dir / "session_manifest.json"
        temporary = output_dir / ".session_manifest.json.tmp"
        temporary.write_text(
            json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)
        return destination
