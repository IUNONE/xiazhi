from __future__ import annotations

import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Iterable, Iterator
from contextlib import contextmanager
from pathlib import Path

from data_pipeline.data_collection.config import CameraConfig, CollectionConfig
from data_pipeline.data_collection.interaction import TerminalKeyReader
from data_pipeline.data_collection.manifest import SessionManifest
from data_pipeline.data_collection.realman_ros import RealManRosArm


class Rosbag2Collector:
    """编排官方 ROS 2 驱动并录制多个 rosbag2 episode。"""

    def __init__(self, config: CollectionConfig) -> None:
        config.validate()
        self.config = config
        self._processes: list[subprocess.Popen[str]] = []
        self._arm = RealManRosArm(
            config.robot,
            startup_timeout_s=config.startup_timeout_s,
            sample_timeout_s=config.sample_timeout_s,
        )

    @staticmethod
    def _robot_commands() -> tuple[list[str], ...]:
        """返回机械臂驱动和状态发布器的最小启动命令。"""
        return (
            ["ros2", "launch", "rm_driver", "rm_eco63_driver.launch.py"],
            [
                "ros2",
                "launch",
                "rm_description",
                "rm_eco63_6fb_display.launch.py",
            ],
        )

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
        """在独立进程组中启动受父进程生命周期约束的命令。"""
        guarded_command = [
            sys.executable,
            "-m",
            "data_pipeline.data_collection.process_guard",
            str(os.getpid()),
            *command,
        ]
        process = subprocess.Popen(
            guarded_command,
            text=True,
            start_new_session=True,
        )
        if managed:
            self._processes.append(process)
        return process

    @staticmethod
    def _running_rm_driver_pids() -> tuple[int, ...]:
        """返回当前正在运行的 rm_driver 进程 PID。"""
        result = subprocess.run(
            ["pgrep", "-x", "rm_driver"],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 1:
            return ()
        if result.returncode != 0:
            detail = result.stderr.strip() or f"exit={result.returncode}"
            raise RuntimeError(f"检查 rm_driver 进程失败: {detail}")
        return tuple(int(value) for value in result.stdout.split())

    @classmethod
    def _reject_running_rm_driver(cls) -> None:
        """检测到已有 rm_driver 时拒绝启动新的采集会话。"""
        pids = cls._running_rm_driver_pids()
        if not pids:
            return
        pid_text = ", ".join(str(pid) for pid in pids)
        raise RuntimeError(
            f"检测到已有 rm_driver 进程, 拒绝启动新实例: PID {pid_text}. "
            "请先正常结束旧采集会话并确认进程退出"
        )

    @staticmethod
    @contextmanager
    def _shutdown_signal_guard() -> Iterator[None]:
        """将 SIGTERM 和 SIGHUP 转换为可执行清理流程的中断。"""
        shutdown_requested = False
        previous_handlers: dict[int, signal.Handlers] = {}

        def handle_shutdown(signum: int, frame: object | None) -> None:
            """触发一次 KeyboardInterrupt 并忽略重复终止信号。"""
            del signum, frame
            nonlocal shutdown_requested
            if shutdown_requested:
                return
            shutdown_requested = True
            raise KeyboardInterrupt

        for shutdown_signal in (signal.SIGTERM, signal.SIGHUP):
            previous_handlers[shutdown_signal] = signal.signal(
                shutdown_signal,
                handle_shutdown,
            )
        try:
            yield
        finally:
            for shutdown_signal, previous_handler in previous_handlers.items():
                signal.signal(shutdown_signal, previous_handler)

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

    @staticmethod
    def _wait_recorder_interval(
        recorder: subprocess.Popen[str],
        duration_s: float,
        keys: TerminalKeyReader | None = None,
    ) -> str | None:
        """等待录包缓冲时间并持续检查 recorder 和可选停止键。"""
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if recorder.poll() is not None:
                raise RuntimeError(
                    f"ros2 bag record 提前退出，exit={recorder.returncode}"
                )
            timeout_s = min(0.05, max(0.0, deadline - time.monotonic()))
            if keys is None:
                time.sleep(timeout_s)
                continue
            try:
                command = keys.recording_command(timeout_s=timeout_s)
            except KeyboardInterrupt:
                return "ctrl_c"
            if command is not None:
                return command
        return None

    @staticmethod
    def _max_joint_error_deg(
        actual: Iterable[float],
        expected: Iterable[float],
    ) -> float:
        """计算两组六关节弧度位置之间的最大绝对角度误差。"""
        actual_values = list(actual)
        expected_values = list(expected)
        if len(actual_values) != 6 or len(expected_values) != 6:
            raise ValueError("关节位置必须包含 6 个元素")
        return math.degrees(
            max(
                abs(float(actual_value) - float(expected_value))
                for actual_value, expected_value in zip(
                    actual_values,
                    expected_values,
                    strict=True,
                )
            )
        )

    def _run_replay_with_monitor(
        self,
        recorder: subprocess.Popen[str],
        keys: TerminalKeyReader,
    ) -> str:
        """在后台阻塞复现轨迹并在主线程监控停止键和 recorder。"""
        outcomes: queue.Queue[BaseException | None] = queue.Queue(maxsize=1)

        def replay_worker() -> None:
            """执行控制器阻塞轨迹复现并传回结果。"""
            try:
                self._arm.run_drag_trajectory()
            except BaseException as exc:
                outcomes.put(exc)
            else:
                outcomes.put(None)

        worker = threading.Thread(
            target=replay_worker,
            name="realman-drag-replay",
            daemon=True,
        )
        worker.start()
        stop_command: str | None = None
        while worker.is_alive() and not self._arm.is_replay_active:
            time.sleep(0.01)

        while worker.is_alive():
            if recorder.poll() is not None:
                recorder_error = RuntimeError(
                    f"ros2 bag record 提前退出，exit={recorder.returncode}"
                )
                try:
                    self._arm.stop_drag_trajectory()
                except BaseException as stop_error:
                    raise RuntimeError(
                        f"{recorder_error}；停止轨迹复现也失败: {stop_error}"
                    ) from stop_error
                worker.join(timeout=10.0)
                if worker.is_alive():
                    raise RuntimeError(f"{recorder_error}；轨迹复现线程未在 10 秒内结束")
                raise recorder_error

            try:
                stop_command = keys.recording_command(timeout_s=0.1)
            except KeyboardInterrupt:
                stop_command = "ctrl_c"
            if stop_command is not None:
                self._arm.stop_drag_trajectory()
                worker.join(timeout=10.0)
                if worker.is_alive():
                    raise RuntimeError("停止指令成功后轨迹复现线程未在 10 秒内结束")
                return stop_command

        worker.join()
        outcome = outcomes.get_nowait()
        if outcome is not None:
            raise outcome
        return "replay_completed"

    def _collect_direct_session(
        self,
        session_path: Path,
        manifest: SessionManifest,
        keys: TerminalKeyReader,
    ) -> str:
        """执行原有边拖动边录包的多 episode 采集状态机。"""
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
            while True:
                try:
                    drag_command = keys.wait_for_drag_start(
                        f"direct episode {episode_index:06d}"
                    )
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if drag_command == "q":
                    session_reason = "q_key"
                    break
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
                current_episode["capture_mode"] = "direct"
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
            manifest.write(session_path)

        if failure is not None:
            raise failure
        if cleanup_failure is not None:
            raise cleanup_failure
        return session_reason

    def _collect_teach_replay_session(
        self,
        session_path: Path,
        manifest: SessionManifest,
        keys: TerminalKeyReader,
    ) -> str:
        """执行先示教再由控制器复现并录包的多 episode 状态机。"""
        recorder: subprocess.Popen[str] | None = None
        current_episode: dict | None = None
        current_attempt: dict | None = None
        drag_started = False
        attempt_index = 0
        episode_index = 0
        session_reason = "q_key"
        failure: BaseException | None = None
        cleanup_failure: BaseException | None = None

        try:
            while True:
                trajectory_path = self.config.trajectory_output_path(attempt_index)
                if trajectory_path.exists():
                    raise FileExistsError(f"轨迹输出路径已存在: {trajectory_path}")
                try:
                    drag_command = keys.wait_for_drag_start(
                        f"示教 attempt {attempt_index:06d}"
                    )
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if drag_command == "q":
                    session_reason = "q_key"
                    break
                current_attempt = manifest.start_teaching_attempt(
                    attempt_index,
                    trajectory_path,
                )
                teaching_start = self._arm.read_observation().joint_position.copy()
                current_attempt["start_joint_position_rad"] = teaching_start.tolist()
                manifest.write(session_path)

                self._arm.enable_drag_teach()
                drag_started = True
                try:
                    teaching_command = keys.wait_for_teaching_finish(attempt_index)
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if teaching_command == "q":
                    session_reason = "q_key"
                    break

                self._arm.stop_drag_teach()
                drag_started = False
                teaching_end = self._arm.read_observation().joint_position.copy()
                current_attempt["end_joint_position_rad"] = teaching_end.tolist()
                point_count = self._arm.save_drag_trajectory(trajectory_path)
                self._ensure_processes_alive()
                self._check_fresh_samples()
                print(
                    "轨迹保存后检查: rm_driver 和设备 topic 均保持正常。",
                    flush=True,
                )
                manifest.mark_teaching_ready(current_attempt, point_count)
                manifest.write(session_path)

                try:
                    origin_command = keys.wait_for_origin_command()
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if origin_command == "q":
                    session_reason = "q_key"
                    manifest.finish_teaching_attempt(
                        current_attempt,
                        status="cancelled",
                        stop_reason=session_reason,
                    )
                    current_attempt = None
                    manifest.write(session_path)
                    break

                self._arm.move_to_drag_trajectory_origin()
                origin_state = self._arm.read_observation().joint_position
                origin_error_deg = self._max_joint_error_deg(
                    origin_state,
                    teaching_start,
                )
                current_attempt["origin_max_joint_error_deg"] = origin_error_deg
                if origin_error_deg > self.config.robot.home_tolerance_deg:
                    raise RuntimeError(
                        f"轨迹起点关节误差 {origin_error_deg:.3f} deg 超过 "
                        f"{self.config.robot.home_tolerance_deg:.3f} deg，"
                        "拒绝回放疑似旧轨迹"
                    )
                manifest.status = "waiting_for_replay"
                manifest.write(session_path)

                try:
                    replay_command = keys.wait_for_replay_command()
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if replay_command == "d":
                    manifest.finish_teaching_attempt(
                        current_attempt,
                        status="discarded",
                        stop_reason="discard_key",
                    )
                    current_attempt = None
                    manifest.write(session_path)
                    attempt_index += 1
                    continue
                if replay_command == "q":
                    session_reason = "q_key"
                    manifest.finish_teaching_attempt(
                        current_attempt,
                        status="cancelled",
                        stop_reason=session_reason,
                    )
                    current_attempt = None
                    manifest.write(session_path)
                    break

                episode_path = self.config.episode_output_path(episode_index)
                if episode_path.exists():
                    raise FileExistsError(f"episode 输出路径已存在: {episode_path}")
                current_episode = manifest.start_episode(episode_index, episode_path)
                current_episode.update(
                    {
                        "capture_mode": "teach-replay",
                        "teaching_attempt_index": attempt_index,
                        "trajectory": {
                            "path": str(trajectory_path),
                            "point_count": point_count,
                            "source": "realman_controller_drag_buffer",
                        },
                        "replay": {
                            "status": "starting",
                            "pre_roll_s": self.config.replay_pre_roll_s,
                            "post_roll_s": self.config.replay_post_roll_s,
                        },
                    }
                )
                current_attempt["episode_index"] = episode_index
                manifest.write(session_path)

                recorder = self._start(
                    self._record_command(episode_path),
                    managed=False,
                )
                self._wait_for_recorder_start(recorder, episode_path)
                print(
                    f"episode {episode_index:06d} recorder 已启动，"
                    f"等待 {self.config.replay_pre_roll_s:.3f} 秒 pre-roll。",
                    flush=True,
                )
                replay_result = self._wait_recorder_interval(
                    recorder,
                    self.config.replay_pre_roll_s,
                    keys,
                )
                if replay_result is None:
                    current_episode["replay"]["status"] = "running"
                    manifest.write(session_path)
                    print(
                        "机械臂即将复现轨迹，rosbag2 正在录制。"
                        "按 s 停止当前回放；按 q 或 Ctrl-C 结束会话。",
                        flush=True,
                    )
                    replay_result = self._run_replay_with_monitor(recorder, keys)

                if replay_result == "replay_completed":
                    replay_end_state = self._arm.read_observation().joint_position
                    replay_end_error_deg = self._max_joint_error_deg(
                        replay_end_state,
                        teaching_end,
                    )
                    current_episode["replay"][
                        "end_max_joint_error_deg"
                    ] = replay_end_error_deg
                    if replay_end_error_deg > self.config.robot.home_tolerance_deg:
                        raise RuntimeError(
                            f"轨迹回放终点关节误差 {replay_end_error_deg:.3f} deg 超过 "
                            f"{self.config.robot.home_tolerance_deg:.3f} deg"
                        )
                    self._wait_recorder_interval(
                        recorder,
                        self.config.replay_post_roll_s,
                    )
                self._stop_process(recorder)
                recorder = None

                if replay_result == "replay_completed":
                    current_episode["replay"]["status"] = "completed"
                    manifest.finish_episode(
                        current_episode,
                        status="completed",
                        stop_reason=replay_result,
                    )
                    manifest.finish_teaching_attempt(
                        current_attempt,
                        status="replayed",
                        stop_reason=replay_result,
                        episode_index=episode_index,
                    )
                else:
                    replay_error = RuntimeError(f"轨迹复现被 {replay_result} 中止")
                    current_episode["replay"]["status"] = "aborted"
                    manifest.finish_episode(
                        current_episode,
                        status="failed",
                        stop_reason=replay_result,
                        error=replay_error,
                    )
                    manifest.finish_teaching_attempt(
                        current_attempt,
                        status="replay_aborted",
                        stop_reason=replay_result,
                        episode_index=episode_index,
                        error=replay_error,
                    )

                current_episode = None
                current_attempt = None
                manifest.write(session_path)
                episode_index += 1
                attempt_index += 1

                if replay_result in {"q", "ctrl_c"}:
                    session_reason = replay_result
                    break

                try:
                    home_command = keys.wait_for_home_command()
                except KeyboardInterrupt:
                    session_reason = "ctrl_c"
                    break
                if home_command == "q":
                    session_reason = "q_key"
                    break
                self._arm.restore_home()
                manifest.status = "ready"
                manifest.write(session_path)
        except KeyboardInterrupt:
            session_reason = "ctrl_c"
        except BaseException as exc:
            session_reason = "error"
            failure = exc
        finally:
            if recorder is not None:
                self._stop_process(recorder)
            if self._arm.is_replay_active:
                try:
                    self._arm.stop_drag_trajectory()
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            if drag_started or self._arm.is_drag_enabled:
                try:
                    self._arm.stop_drag_teach()
                    drag_started = False
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            final_error = failure or cleanup_failure
            attempt_episode_index = (
                current_episode["episode_index"]
                if current_episode is not None
                else None
            )
            if current_episode is not None:
                current_episode["replay"]["status"] = "failed"
                manifest.finish_episode(
                    current_episode,
                    status="failed",
                    stop_reason=session_reason,
                    error=final_error or RuntimeError("episode 未正常结束"),
                )
            if current_attempt is not None:
                manifest.finish_teaching_attempt(
                    current_attempt,
                    status="failed" if final_error is not None else "cancelled",
                    stop_reason=session_reason,
                    episode_index=attempt_episode_index,
                    error=final_error,
                )
            manifest.write(session_path)

        if failure is not None:
            raise failure
        if cleanup_failure is not None:
            raise cleanup_failure
        return session_reason

    def _collect_session(self) -> Path:
        """启动设备并按配置的采集模式生成一个多 episode session。"""
        session_path = self.config.output_path()
        if session_path.exists():
            raise FileExistsError(f"输出 session 路径已存在: {session_path}")
        session_path.mkdir(parents=True, exist_ok=False)

        manifest = SessionManifest.from_config(self.config, backend="rosbag2")
        manifest.status = "starting"
        manifest.write(session_path)
        session_reason = "q_key"
        failure: BaseException | None = None
        cleanup_failure: BaseException | None = None

        try:
            with TerminalKeyReader() as keys:
                for command in self._robot_commands():
                    self._start(command)
                for camera in self.config.cameras:
                    self._start(self._camera_command(camera))
                self._wait_for_topics()
                self._check_fresh_samples()
                self._arm.connect()
                if self.config.robot.init_joints_deg is not None:
                    print(
                        "机械臂状态: 正在运动到采集初始关节位姿, 不录制 rosbag2; "
                        f"joints_deg={list(self.config.robot.init_joints_deg)}",
                        flush=True,
                    )
                    self._arm.restore_home()
                    print("机械臂状态: 已到达采集初始关节位姿", flush=True)
                self._arm.read_observation()
                if self.config.capture_mode == "direct":
                    session_reason = self._collect_direct_session(
                        session_path,
                        manifest,
                        keys,
                    )
                else:
                    session_reason = self._collect_teach_replay_session(
                        session_path,
                        manifest,
                        keys,
                    )
        except KeyboardInterrupt:
            session_reason = "ctrl_c"
        except BaseException as exc:
            session_reason = "error"
            failure = exc
        finally:
            if self._arm.is_replay_active:
                try:
                    self._arm.stop_drag_trajectory()
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
            if self._arm.is_drag_enabled:
                try:
                    self._arm.stop_drag_teach()
                except BaseException as exc:
                    cleanup_failure = cleanup_failure or exc
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

    def collect(self) -> Path:
        """拒绝冲突驱动并在受管信号范围内执行采集会话。"""
        self._reject_running_rm_driver()
        with self._shutdown_signal_guard():
            return self._collect_session()
