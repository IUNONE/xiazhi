"""Zhongling ASCII feedback adapter, based on MINT-SJTU U-Arm's protocol usage.

Reference: src/uarm/scripts/Uarm_teleop/Zhonglin_servo/servo_reader.py
No ROS1 dependency, persistent calibration writes or implicit zero fallback.
"""
from __future__ import annotations

import math
import re
import threading
import time
from dataclasses import dataclass

from data_pipeline.data_collection.teleoperation.config import Calibration, LeaderConfig, RuntimeSettings


@dataclass(frozen=True)
class LeaderSample:
    timestamp_ns: int  # monotonic; oldest channel in the scan
    positions: tuple[float, ...]  # rad; servo_ids order
    raw: tuple[int, ...]


class ZhonglingLeader:
    def __init__(self, config: LeaderConfig, calibration: Calibration) -> None:
        self.config, self.calibration = config, calibration
        self._serial = None
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._io_lock = threading.RLock()
        self._homing_active = False
        self._latest = None
        self._failure = None

    def connect(self) -> None:
        import serial
        self._serial = serial.Serial(self.config.port, self.config.baudrate, timeout=0.05, write_timeout=0.1)
        try:
            self._latest = self.read_once()
            self._thread = threading.Thread(target=self._run, name="zhongling-reader", daemon=True)
            self._thread.start()
        except BaseException:
            self.disconnect()
            raise

    def read_once(self) -> LeaderSample:
        with self._io_lock:
            sample = self._read_once()
            with self._lock:
                self._latest = sample
            return sample

    def _read_once(self) -> LeaderSample:
        if self._serial is None:
            raise RuntimeError("主臂串口未连接")
        started = time.monotonic_ns()
        raw_values, positions = [], []
        c = self.calibration
        for servo, lo, hi, angle in zip(c.servo_ids,c.raw_min,c.raw_max,c.angle_range_deg):
            self._serial.reset_input_buffer()
            self._serial.write(f"#{servo:03d}PRAD!".encode("ascii"))
            pattern = re.compile(rb"#" + f"{servo:03d}".encode() + rb"P(\d{4})(?!\d)")
            response = bytearray()
            deadline = time.monotonic() + 0.1
            match = None
            while time.monotonic() < deadline and len(response) < 256:
                response.extend(self._serial.read(max(1, self._serial.in_waiting)))
                match = pattern.search(response)
                if match:
                    break
            if match is None:
                raise TimeoutError(f"舵机 {servo} 未返回有效位置: {bytes(response)!r}")
            raw = int(match.group(1))
            if not lo <= raw <= hi:
                raise ValueError(f"舵机 {servo} 反馈 {raw} 超出标定范围")
            raw_values.append(raw)
            positions.append(math.radians((raw-lo)/(hi-lo)*angle))
        if (time.monotonic_ns()-started)/1e9 > self.config.stale_after_s:
            raise TimeoutError("主臂完整扫描超过数据有效期")
        return LeaderSample(started, tuple(positions), tuple(raw_values))

    def _write(self, command: str) -> None:
        if self._serial is None:
            raise RuntimeError("主臂未连接")
        payload = command.encode("ascii")
        if self._serial.write(payload) != len(payload):
            raise IOError("主臂指令未完整发送")

    def release_torque(self) -> None:
        """Called only after the operator confirms supporting the leader."""
        with self._io_lock:
            for servo in self.calibration.servo_ids:
                self._write(f"#{servo:03d}PULK!")
                time.sleep(0.01)

    def stop_motion(self) -> None:
        home = RuntimeSettings.load(self.config.runtime_config).homing(self.calibration)
        with self._io_lock:
            for servo in self.calibration.servo_ids:
                self._write(home['stop_command'].format(id=servo))
        self._homing_active = False

    def move_home(self, poll=None) -> None:
        home = RuntimeSettings.load(self.config.runtime_config).homing(self.calibration)
        sample = self.read_once()
        duration = home['duration_ms']
        if any(abs(target-current)/(duration/1000) > home['max_speed_raw_s']
               for target, current in zip(home['raw'], sample.raw)):
            raise ValueError("主臂回位距离超过设定速度，请增加 duration_ms 或人工调整起点")
        self._homing_active = True
        try:
            with self._io_lock:
                # The verified position command must re-enable holding at the
                # requested position, without first returning to an old target.
                for servo, current in zip(self.calibration.servo_ids, sample.raw):
                    self._write(home['position_command'].format(id=servo, position=current, duration=duration))
                for servo, target in zip(self.calibration.servo_ids, home['raw']):
                    self._write(home['position_command'].format(id=servo, position=target, duration=duration))
            started = time.monotonic()
            stable = 0
            previous_stamp = None
            while time.monotonic()-started < home['timeout_s']:
                if poll is not None:
                    poll()
                sample = self.latest()
                if sample.timestamp_ns != previous_stamp:
                    stable = stable+1 if all(abs(a-b) <= home['tolerance_raw'] for a,b in zip(sample.raw, home['raw'])) else 0
                    previous_stamp = sample.timestamp_ns
                if stable >= 3 and time.monotonic()-started >= duration/1000:
                    self._homing_active = False
                    return
                time.sleep(0.02)
            raise TimeoutError("主臂回位未在规定时间内到位")
        except BaseException:
            self.stop_motion()
            raise

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self.read_once()
        except BaseException as exc:
            with self._lock:
                self._failure = exc

    def latest(self) -> LeaderSample:
        with self._lock:
            if self._failure is not None:
                raise RuntimeError("主臂读取失败") from self._failure
            sample = self._latest
        if sample is None or (time.monotonic_ns()-sample.timestamp_ns)/1e9 > self.config.stale_after_s:
            raise TimeoutError("主臂数据过期，停止跟随")
        return sample

    def disconnect(self) -> None:
        try:
            if self._homing_active:
                self.stop_motion()
        finally:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2)
            if self._serial is not None:
                self._serial.close()
                self._serial = None
