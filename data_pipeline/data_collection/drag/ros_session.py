"""Existing kinesthetic-teaching and controller-replay episode workflows."""
from __future__ import annotations

import subprocess
from pathlib import Path

from data_pipeline.data_collection.io_utils.manifest import SessionManifest
from data_pipeline.data_collection.io_utils.terminal import TerminalKeyReader


def collect_direct_session(
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

def collect_teach_replay_session(
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

