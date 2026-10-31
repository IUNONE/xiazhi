from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

from data_pipeline.data_collection.hardware.zhongling import ZhonglingLeader
from data_pipeline.data_collection.teleoperation.config import Calibration, LeaderConfig


@dataclass(frozen=True)
class CommandSample:
    sequence: int
    timestamp_ns: int  # monotonic, command submission start
    wall_timestamp_ns: int
    leader_timestamp_ns: int
    leader_position: tuple[float, ...]
    target: tuple[float, ...]


class TeleoperationController:
    """One command owner; low-follow ROS targets, independent of bag recording.

    The recorder must heartbeat. Commands are bounded and timestamped; failures
    stop the arm and propagate to the recording session. No automatic homing.
    """
    def __init__(self, arm, leader: ZhonglingLeader, config: LeaderConfig, calibration: Calibration, limits, on_command=None):
        self.arm, self.leader, self.config, self.calibration = arm, leader, config, calibration
        self.on_command = on_command
        self.limits = limits
        self._stop = threading.Event()
        self._thread = None
        self._failure = None
        self._heartbeat = time.monotonic()

    def start(self) -> None:
        initial = self.arm.latest_observation().joint_position.astype(float)
        c = self.calibration
        if initial.shape != (6,) or not np.isfinite(initial).all() or np.any(initial < self.limits.lower_rad) or np.any(initial > self.limits.upper_rad):
            raise ValueError("从臂当前位置超出 URDF 关节限位")
        desired = c.joint_positions(self.leader.latest().positions)
        if not np.isfinite(desired).all() or np.any(desired < self.limits.lower_rad) or np.any(desired > self.limits.upper_rad):
            raise ValueError("主臂绝对目标超出从臂 URDF 限位或包含无效角度")
        # Actual position seeds only the velocity limiter, never the mapping zero.
        self.initial = initial
        self._thread = threading.Thread(target=self._run, name="leader-follower", daemon=True)
        self._thread.start()

    def heartbeat(self) -> None:
        self._heartbeat = time.monotonic()
        self.check()

    def check(self) -> None:
        if self._failure is not None:
            raise RuntimeError(f"遥操控制已停止: {self._failure}") from self._failure

    def _run(self) -> None:
        c = self.calibration
        last = self.initial.copy()
        interval = 1 / self.config.control_hz
        deadline = time.monotonic()
        previous_time = deadline
        sequence = 0
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                if now-self._heartbeat > 1.0:
                    raise TimeoutError("采集循环超过 1 秒未响应")
                if now-deadline > max(0.2, 2*interval) or (sequence and now-previous_time > max(0.2, 2*interval)):
                    raise TimeoutError("控制循环超时")
                sample = self.leader.latest()
                desired = c.joint_positions(sample.positions)
                if not np.isfinite(desired).all() or np.any(desired < self.limits.lower_rad) or np.any(desired > self.limits.upper_rad):
                    raise ValueError("主臂目标超出从臂关节限位")
                dt = min(max(now-previous_time, 0), interval)
                target = last + np.clip(desired-last, -np.asarray(self.limits.max_velocity_rad_s)*dt, np.asarray(self.limits.max_velocity_rad_s)*dt)
                # Feedback freshness is still required; position lag alone does
                # not abort teleoperation. Reading this cache never waits for a frame.
                self.arm.latest_observation()
                self.leader.latest()
                if self._stop.is_set():
                    break
                if time.monotonic()-self._heartbeat > 1.0:
                    raise TimeoutError("采集循环未响应")
                stamp, wall = time.monotonic_ns(), time.time_ns()
                if (stamp-sample.timestamp_ns)/1e9 > self.config.stale_after_s:
                    raise TimeoutError("发送前主臂样本已过期")
                self.arm.send_joint_target(target)
                command = CommandSample(sequence, stamp, wall, sample.timestamp_ns, sample.positions, tuple(float(v) for v in target))
                if self.on_command is not None:
                    self.on_command(command)
                sequence += 1
                last, previous_time = target, now
                deadline = max(deadline + interval, time.monotonic())
                self._stop.wait(max(0, deadline-time.monotonic()))
        except BaseException as exc:
            self._failure = exc
        finally:
            try:
                self.arm.stop_motion()
            except BaseException as exc:
                self._failure = self._failure or exc

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise TimeoutError("遥操线程未停止；拒绝继续操作设备")
        self.check()
