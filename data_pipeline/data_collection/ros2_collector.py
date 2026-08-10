from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path

from data_pipeline.data_collection.config import CameraConfig, CollectionConfig
from data_pipeline.data_collection.interaction import TerminalKeyReader
from data_pipeline.data_collection.manifest import SessionManifest
from data_pipeline.data_collection.realman import RealManArm


class Rosbag2Collector:
    """Launch official ROS 2 drivers and record multiple rosbag2 episodes."""

    def __init__(self, config: CollectionConfig) -> None:
        config.validate()
        if not config.allow_sdk_driver_coexistence:
            raise ValueError(
                "ROS 驱动未暴露 multi-drag-teach 接口。"
                "必须先完成同机并发连接真机验证，"
                "然后显式设置 allow_sdk_driver_coexistence=True"
            )
        self.config = config
        self._processes: list[subprocess.Popen[str]] = []
        self._arm = RealManArm(config.robot)

    @staticmethod
    def _robot_command() -> list[str]:
        return ["ros2", "launch", "rm_bringup", "rm_eco63_6fb_bringup.launch.py"]

    @staticmethod
    def _camera_command(camera: CameraConfig) -> list[str]:
        return [
            "ros2",
            "launch",
            "orbbec_camera",
            "gemini_330_series.launch.py",
            f"camera_name:={camera.name}",
            f"serial_number:={camera.serial_number}",
            f"color_width:={camera.width}",
            f"color_height:={camera.height}",
            f"color_fps:={camera.fps}",
            f"depth_width:={camera.width}",
            f"depth_height:={camera.height}",
            f"depth_fps:={camera.fps}",
            f"enable_depth:={'true' if camera.enable_depth else 'false'}",
            "enable_point_cloud:=false",
        ]

    def _record_command(self, output_path: Path) -> list[str]:
        return [
            "ros2",
            "bag",
            "record",
            "--storage",
            self.config.storage_id,
            "--output",
            str(output_path),
            *self.config.topics(),
        ]

    def _start(
        self,
        command: list[str],
        *,
        managed: bool = True,
    ) -> subprocess.Popen[str]:
        process = subprocess.Popen(command, text=True, start_new_session=True)
        if managed:
            self._processes.append(process)
        return process

    def _ensure_processes_alive(self) -> None:
        stopped = [process.args for process in self._processes if process.poll() is not None]
        if stopped:
            raise RuntimeError(f"子进程提前退出: {stopped}")

    def _listed_topics(self) -> set[str]:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=True,
            capture_output=True,
            text=True,
            timeout=self.config.sample_timeout_s,
        )
        return set(result.stdout.splitlines())

    def _wait_for_topics(self) -> None:
        expected = set(self.config.topics())
        deadline = time.monotonic() + self.config.startup_timeout_s
        missing = expected
        while time.monotonic() < deadline:
            self._ensure_processes_alive()
            missing = expected - self._listed_topics()
            if not missing:
                return
            time.sleep(0.5)
        raise TimeoutError(f"等待 ROS 2 topic 超时: {sorted(missing)}")

    def _check_topic_type(self, topic: str, expected_type: str) -> None:
        result = subprocess.run(
            ["ros2", "topic", "type", topic],
            check=True,
            capture_output=True,
            text=True,
            timeout=self.config.sample_timeout_s,
        )
        actual = result.stdout.strip()
        if actual != expected_type:
            raise TypeError(f"{topic} 类型错误: {actual!r} != {expected_type!r}")

    def _check_fresh_samples(self) -> None:
        expected_types = {
            "/joint_states": "sensor_msgs/msg/JointState",
            "/rm_driver/udp_six_zero_force": "rm_ros_interfaces/msg/Sixforce",
        }
        for camera in self.config.cameras:
            expected_types[f"/{camera.name}/color/image_raw"] = "sensor_msgs/msg/Image"
            if camera.enable_depth:
                expected_types[f"/{camera.name}/depth/image_raw"] = "sensor_msgs/msg/Image"

        for topic in self.config.fresh_sample_topics():
            self._ensure_processes_alive()
            self._check_topic_type(topic, expected_types[topic])
            try:
                subprocess.run(
                    ["ros2", "topic", "echo", topic, "--once", "--no-arr"],
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=self.config.sample_timeout_s,
                )
            except subprocess.TimeoutExpired as exc:
                raise TimeoutError(f"{topic} 在限定时间内没有新消息") from exc

    @staticmethod
    def _stop_process(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10.0)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def _stop_all(self) -> None:
        for process in reversed(self._processes):
            self._stop_process(process)
        self._processes.clear()

    def _wait_for_recorder_start(self, recorder: subprocess.Popen[str], output_path: Path) -> None:
        deadline = time.monotonic() + self.config.sample_timeout_s
        while time.monotonic() < deadline:
            if recorder.poll() is not None:
                raise RuntimeError(f"ros2 bag record 启动失败，exit={recorder.returncode}")
            if output_path.is_dir():
                return
            time.sleep(0.05)
        raise TimeoutError("ros2 bag record 未及时创建输出目录")

    def collect(self) -> Path:
        session_path = self.config.output_path()
        if session_path.exists():
            raise FileExistsError(f"输出 session 路径已存在: {session_path}")
        session_path.mkdir(parents=True, exist_ok=False)

        manifest = SessionManifest.from_config(self.config, backend="rosbag2")
        manifest.status = "starting"
        manifest.write(session_path)
        recorder: subprocess.Popen[str] | None = None
        recorder_started = False
        current_episode: dict | None = None
        drag_started = False
        needs_restore = False
        episode_index = 0
        session_reason = "q_key"
        failure: BaseException | None = None
        cleanup_failure: BaseException | None = None

        try:
            with TerminalKeyReader() as keys:
                self._start(self._robot_command())
                for camera in self.config.cameras:
                    self._start(self._camera_command(camera))
                self._wait_for_topics()
                self._check_fresh_samples()
                self._arm.connect()
                self._arm.read_observation()

                while True:
                    self._arm.enable_drag_teach()
                    drag_started = True
                    needs_restore = True
                    manifest.status = "ready"
                    manifest.write(session_path)
                    try:
                        ready_command = keys.wait_for_episode_start()
                    except KeyboardInterrupt:
                        session_reason = "ctrl_c"
                        break
                    if ready_command == "q":
                        session_reason = "q_key"
                        break

                    episode_path = self.config.episode_output_path(episode_index)
                    if episode_path.exists():
                        raise FileExistsError(f"episode 输出路径已存在: {episode_path}")
                    current_episode = manifest.start_episode(episode_index, episode_path)
                    manifest.write(session_path)
                    recorder = self._start(
                        self._record_command(episode_path),
                        managed=False,
                    )
                    self._wait_for_recorder_start(recorder, episode_path)
                    recorder_started = True
                    print(
                        f"episode {episode_index:06d} 正在录制。"
                        "按 s 结束 episode；按 q 或 Ctrl-C 结束会话。",
                        flush=True,
                    )

                    recording_command: str | None = None
                    while True:
                        if recorder.poll() is not None:
                            raise RuntimeError(
                                f"ros2 bag record 提前退出，exit={recorder.returncode}"
                            )
                        try:
                            recording_command = keys.recording_command(timeout_s=0.1)
                        except KeyboardInterrupt:
                            recording_command = "ctrl_c"
                        if recording_command is not None:
                            break

                    self._stop_process(recorder)
                    recorder = None
                    self._arm.stop_drag_teach()
                    drag_started = False
                    print(
                        "episode 已停止，机械臂将低速返回采集前位姿。",
                        flush=True,
                    )
                    self._arm.restore_home()
                    needs_restore = False
                    manifest.finish_episode(
                        current_episode,
                        status="completed",
                        stop_reason=recording_command,
                    )
                    current_episode = None
                    recorder_started = False
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
            if recorder is not None:
                self._stop_process(recorder)
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
            if current_episode is not None:
                episode_completed = (
                    recorder_started
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
                    error=episode_error,
                )
            try:
                self._arm.disconnect()
            except BaseException as exc:
                cleanup_failure = cleanup_failure or exc
            self._stop_all()

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
