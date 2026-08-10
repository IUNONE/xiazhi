from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from data_pipeline.data_collection.config import CollectionConfig
from data_pipeline.data_collection.interaction import TerminalKeyReader
from data_pipeline.data_collection.manifest import SessionManifest
from data_pipeline.data_collection.orbbec import OrbbecCamera
from data_pipeline.data_collection.realman import RealManArm
from data_pipeline.data_format.alignment import next_joint_target
from data_pipeline.data_format.schema import build_lerobot_features


class LeRobotCollector:
    """Record multiple kinesthetic episodes into one LeRobotDataset."""

    def __init__(
        self,
        config: CollectionConfig,
        *,
        repo_id: str,
        use_videos: bool = True,
        max_camera_age_ms: int = 500,
    ) -> None:
        config.validate()
        if not repo_id or "/" not in repo_id:
            raise ValueError("repo_id 必须使用 namespace/name 格式")
        if max_camera_age_ms <= 0:
            raise ValueError("max_camera_age_ms 必须大于 0")
        self.config = config
        self.repo_id = repo_id
        self.use_videos = use_videos
        self.max_camera_age_ms = max_camera_age_ms
        self._arm = RealManArm(config.robot)
        self._cameras = [OrbbecCamera(camera) for camera in config.cameras]

    @staticmethod
    def _dataset_class():
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise ImportError(
                "LeRobot 直采需要 Python 3.12 和 third_party/lerobot[dataset_viz]"
            ) from exc
        return LeRobotDataset

    def _observation(self) -> dict[str, Any]:
        arm = self._arm.read_observation()
        frame: dict[str, Any] = {
            "observation.state": arm.joint_position,
            "observation.ee_pose": arm.ee_pose,
            "observation.force": arm.force,
            "task": self.config.task,
        }
        for camera in self._cameras:
            rgbd = camera.read_latest_rgbd(max_age_ms=self.max_camera_age_ms)
            frame[f"observation.images.{camera.config.name}"] = rgbd.color
            if rgbd.depth_mm is not None:
                frame[f"observation.images.{camera.config.name}_depth"] = rgbd.depth_mm[
                    ..., None
                ]
        return frame

    @staticmethod
    def _paired_frame(
        previous: dict[str, Any],
        current: dict[str, Any],
    ) -> dict[str, Any]:
        frame = dict(previous)
        frame["action"] = next_joint_target(current)
        return frame

    @staticmethod
    def _wait_until(deadline: float, keys: TerminalKeyReader) -> str | None:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            command = keys.recording_command(timeout_s=min(remaining, 0.05))
            if command is not None:
                return command

    def collect(self) -> Path:
        session_path = self.config.output_path()
        if session_path.exists():
            raise FileExistsError(f"输出 session 路径已存在: {session_path}")
        session_path.parent.mkdir(parents=True, exist_ok=True)
        dataset_class = self._dataset_class()

        manifest = SessionManifest.from_config(self.config, backend="lerobot")
        dataset = None
        current_episode: dict | None = None
        current_frame_count = 0
        episode_committed = False
        drag_started = False
        needs_restore = False
        episode_index = 0
        session_reason = "q_key"
        failure: BaseException | None = None
        cleanup_failure: BaseException | None = None

        try:
            with TerminalKeyReader() as keys:
                self._arm.connect()
                for camera in self._cameras:
                    camera.connect()
                self._arm.read_observation()
                for camera in self._cameras:
                    camera.read_latest_rgbd(max_age_ms=self.max_camera_age_ms)

                while True:
                    self._arm.enable_drag_teach()
                    drag_started = True
                    needs_restore = True
                    manifest.status = "ready"
                    if session_path.exists():
                        manifest.write(session_path)
                    try:
                        ready_command = keys.wait_for_episode_start()
                    except KeyboardInterrupt:
                        session_reason = "ctrl_c"
                        break
                    if ready_command == "q":
                        session_reason = "q_key"
                        break

                    current_episode = manifest.start_episode(episode_index, session_path)
                    current_frame_count = 0
                    episode_committed = False
                    previous = self._observation()
                    interval_s = 1.0 / self.config.fps
                    next_deadline = time.monotonic() + interval_s
                    print(
                        f"episode {episode_index:06d} 正在录制。"
                        "按 s 结束 episode；按 q 或 Ctrl-C 结束会话。",
                        flush=True,
                    )

                    recording_command: str | None = None
                    while True:
                        try:
                            recording_command = self._wait_until(next_deadline, keys)
                        except KeyboardInterrupt:
                            recording_command = "ctrl_c"
                        if recording_command is not None:
                            break

                        current = self._observation()
                        frame = self._paired_frame(previous, current)
                        if dataset is None:
                            dataset = dataset_class.create(
                                repo_id=self.repo_id,
                                fps=self.config.fps,
                                features=build_lerobot_features(
                                    frame,
                                    use_videos=self.use_videos,
                                ),
                                root=session_path,
                                robot_type="realman_eco63_6fb",
                                use_videos=self.use_videos,
                                image_writer_threads=4,
                            )
                            manifest.write(session_path)
                        dataset.add_frame(frame)
                        current_frame_count += 1
                        previous = current
                        next_deadline += interval_s
                        if next_deadline < time.monotonic():
                            next_deadline = time.monotonic() + interval_s

                    self._arm.stop_drag_teach()
                    drag_started = False
                    print(
                        "episode 已停止，机械臂将低速返回采集前位姿。",
                        flush=True,
                    )
                    self._arm.restore_home()
                    needs_restore = False
                    if dataset is None or current_frame_count == 0:
                        raise ValueError(
                            "episode 少于两个采样点，无法构造下一关节目标 action"
                        )
                    dataset.save_episode()
                    episode_committed = True
                    manifest.finish_episode(
                        current_episode,
                        status="completed",
                        stop_reason=recording_command,
                        frame_count=current_frame_count,
                    )
                    current_episode = None
                    current_frame_count = 0
                    episode_committed = False
                    manifest.write(session_path)
                    episode_index += 1

                    if recording_command in {"q", "ctrl_c"}:
                        session_reason = recording_command
                        break
        except KeyboardInterrupt:
            session_reason = "ctrl_c"
        except BaseException as exc:
            session_reason = "error"
            failure = exc
        finally:
            if drag_started:
                try:
                    self._arm.stop_drag_teach()
                    drag_started = False
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            if self._arm.is_connected and needs_restore:
                try:
                    print(
                        "采集会话结束，机械臂将低速返回采集前位姿。",
                        flush=True,
                    )
                    self._arm.restore_home()
                    needs_restore = False
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            for camera in reversed(self._cameras):
                try:
                    camera.disconnect()
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            try:
                self._arm.disconnect()
            except BaseException as exc:
                cleanup_failure = cleanup_failure or exc

            if current_episode is not None:
                if dataset is not None and dataset.has_pending_frames():
                    try:
                        dataset.save_episode()
                        episode_committed = True
                    except BaseException as exc:
                        cleanup_failure = cleanup_failure or exc
                episode_completed = (
                    episode_committed
                    and current_frame_count > 0
                    and failure is None
                    and cleanup_failure is None
                    and session_reason == "ctrl_c"
                )
                episode_error = None
                if not episode_completed:
                    episode_error = (
                        failure or cleanup_failure or RuntimeError("episode 未正常结束")
                    )
                    cleanup_failure = cleanup_failure or episode_error
                manifest.finish_episode(
                    current_episode,
                    status="completed" if episode_completed else "failed",
                    stop_reason="ctrl_c" if episode_completed else "error",
                    frame_count=current_frame_count,
                    error=episode_error,
                )
            if dataset is not None:
                try:
                    dataset.finalize()
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc

            final_error = failure or cleanup_failure
            manifest.finish_session(
                status="completed" if final_error is None else "failed",
                stop_reason=session_reason,
                error=final_error,
            )
            manifest.write(session_path)

        if failure is not None:
            raise failure
        if cleanup_failure is not None:
            raise cleanup_failure
        return session_path
