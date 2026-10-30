from __future__ import annotations

import hashlib
import json
import math
import string
from dataclasses import dataclass
from pathlib import Path

COMMAND_TOPIC = "/xiazhi/teleop/joint_target"
LEADER_TOPIC = "/xiazhi/teleop/leader_state"


@dataclass(frozen=True)
class LeaderConfig:
    port: str
    calibration: Path
    baudrate: int = 115200
    control_hz: float = 30.0
    stale_after_s: float = 0.3
    runtime_config: Path | None = None
    model_calibration: Path | None = None
    mapping: Path | None = None

    def validate(self) -> None:
        if not self.port or self.baudrate <= 0:
            raise ValueError("主臂串口和波特率无效")
        if not math.isfinite(self.control_hz) or not 1 <= self.control_hz <= 50:
            raise ValueError("低跟随 control_hz 必须在 1..50 Hz")
        if not math.isfinite(self.stale_after_s) or not 0.1 <= self.stale_after_s <= 1:
            raise ValueError("主臂数据有效期必须在 0.1..1 秒")
        from data_pipeline.data_collection.teleoperation.mapping import load_absolute_calibration
        calibration = load_absolute_calibration(self)
        RuntimeSettings.load(self.runtime_config).homing(calibration)


@dataclass(frozen=True)
class Calibration:
    servo_ids: tuple[int, ...]
    raw_min: tuple[float, ...]
    raw_max: tuple[float, ...]
    angle_range_deg: tuple[float, ...]
    joint_indices: tuple[int, ...]
    direction: tuple[int, ...]
    scale: tuple[float, ...]
    sha256: str
    document: dict

    @classmethod
    def load(cls, path: Path, *, require_verified: bool = True) -> Calibration:
        raw = path.read_bytes()
        d = json.loads(raw)
        if d.get("version") not in (1, 2) or (require_verified and d.get("verified") is not True):
            raise ValueError("标定必须为 version=1/2，并在实机核对后设置 verified=true")
        ids = d["servo_ids"]
        if len(ids) not in (6, 7) or len(set(ids)) != len(ids) or any(type(i) is not int or not 0 <= i <= 999 for i in ids):
            raise ValueError("servo_ids 必须包含 6 或 7 个唯一 ID")
        for key in ("raw_min", "raw_max", "angle_range_deg"):
            if len(d[key]) != len(ids) or not all(math.isfinite(float(v)) for v in d[key]):
                raise ValueError(f"无效标定字段: {key}")
        if any(lo >= hi or angle <= 0 for lo, hi, angle in zip(d['raw_min'], d['raw_max'], d['angle_range_deg'])):
            raise ValueError("原始反馈范围和角度范围无效")
        for key in ("joint_indices", "direction", "scale"):
            if len(d[key]) != 6 or not all(math.isfinite(float(v)) for v in d[key]):
                raise ValueError(f"{key} 必须包含六个有限数值")
        if len(set(d['joint_indices'])) != 6 or any(type(i) is not int or not 0 <= i < len(ids) for i in d['joint_indices']):
            raise ValueError("joint_indices 必须指向六个不同主臂通道")
        if any(v not in (-1, 1) for v in d['direction']):
            raise ValueError("direction 只能是 -1 或 1")
        if any(v <= 0 for v in d['scale']):
            raise ValueError("scale 必须为正数")
        zero = d.get('zero_raw')
        if zero is not None:
            if len(zero) != len(ids) or any(not math.isfinite(v) or not lo <= v <= hi for v,lo,hi in zip(zero,d['raw_min'],d['raw_max'])):
                raise ValueError("zero_raw 无效")
        expected = d.get('zero_joints_deg', [0]*6)
        if len(expected) != 6 or not all(math.isfinite(v) for v in expected):
            raise ValueError("zero_joints_deg 无效")
        if require_verified and zero is None:
            raise ValueError("缺少 zero_raw，请先完成主臂模型标定")
        return cls(*(tuple(d[k]) for k in ('servo_ids','raw_min','raw_max','angle_range_deg','joint_indices','direction','scale')), hashlib.sha256(raw).hexdigest(), d)

    def joint_positions(self, positions):
        """Map raw-converted motor angles to calibrated follower joint coordinates."""
        import numpy as np
        zero = self.document.get('zero_raw')
        if zero is None:
            raise ValueError("尚未记录零位")
        zero_angles = np.deg2rad((np.asarray(zero)-self.raw_min)/(np.asarray(self.raw_max)-self.raw_min)*self.angle_range_deg)
        indices = list(self.joint_indices)
        return np.deg2rad(self.document.get('zero_joints_deg', [0]*6)) + np.asarray(self.direction)*self.scale*(np.asarray(positions)[indices]-zero_angles[indices])


@dataclass(frozen=True)
class RuntimeSettings:
    document: dict

    @classmethod
    def load(cls, path: Path | None):
        if path is None:
            path = Path(__file__).resolve().parents[3] / 'configs/teleoperation/eco63_runtime.json'
        document = json.loads(path.read_text())
        if document.get('version') != 1:
            raise ValueError('运行配置 version 必须为 1')
        return cls(document)

    def homing(self, calibration) -> dict:
        """Return explicitly verified leader motion parameters; never infer servo commands."""
        home = dict(self.document.get("homing", {}))
        if home.get("verified") is not True:
            raise ValueError("请先核对运行配置 homing 的主臂回位/停止协议，并设置 verified=true")
        raw = calibration.document.get("home_raw") or calibration.document.get("zero_raw")
        home["raw"] = raw
        if not isinstance(raw, list) or len(raw) != len(calibration.servo_ids):
            raise ValueError("homing.raw 必须按 servo_ids 顺序包含全部主臂初始位置")
        for value, lo, hi in zip(raw, calibration.raw_min, calibration.raw_max):
            if type(value) is not int or not lo <= value <= hi:
                raise ValueError("homing.raw 超出标定范围或不是整数")
        for key in ("duration_ms", "tolerance_raw", "timeout_s", "max_speed_raw_s"):
            value = home.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"homing.{key} 必须为正有限数值")
        if type(home['duration_ms']) is not int or not 1000 <= home['duration_ms'] <= 9999:
            raise ValueError("homing.duration_ms 必须为 1000..9999 的整数毫秒")
        if home['timeout_s'] <= home['duration_ms']/1000:
            raise ValueError("homing.timeout_s 必须大于回位时间")
        for key, required in (("position_command", {"id", "position", "duration"}), ("stop_command", {"id"})):
            template = home.get(key)
            if not isinstance(template, str) or not template.isascii() or any(ord(c) < 32 for c in template):
                raise ValueError(f"homing.{key} 必须是已核对的单行 ASCII 指令模板")
            fields = {field for _, field, _, _ in string.Formatter().parse(template) if field is not None}
            if fields != required:
                raise ValueError(f"homing.{key} 需要占位符 {sorted(required)}")
            rendered = template.format(id=0, position=1500, duration=3000)
            if not rendered or len(rendered) > 128:
                raise ValueError(f"homing.{key} 长度无效")
        return home


    def follower_limits(self):
        import xml.etree.ElementTree as ET
        from data_pipeline.data_rerun.robot_model import DEFAULT_ROBOT_XACRO, build_robot_urdf
        root = ET.fromstring(build_robot_urdf(DEFAULT_ROBOT_XACRO))
        lower, upper, velocity = [], [], []
        factor = float(self.document.get('velocity_scale', 0.1))
        error = float(self.document.get('max_tracking_error_rad', 0.2))
        if not math.isfinite(factor) or not 0 < factor <= 1 or not math.isfinite(error) or error <= 0:
            raise ValueError('运行配置的速度比例或跟随误差无效')
        joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
        for name in (f'joint{i}' for i in range(1,7)):
            limit = joints[name].find('limit')
            lo, hi, speed = (float(limit.attrib[key]) for key in ('lower','upper','velocity'))
            if not all(math.isfinite(v) for v in (lo,hi,speed)) or lo >= hi or speed <= 0:
                raise ValueError(f'URDF 关节限制无效: {name}')
            lower.append(lo); upper.append(hi); velocity.append(speed*factor)
        return FollowerLimits(tuple(lower), tuple(upper), tuple(velocity), error)


@dataclass(frozen=True)
class FollowerLimits:
    lower_rad: tuple[float, ...]
    upper_rad: tuple[float, ...]
    max_velocity_rad_s: tuple[float, ...]
    max_tracking_error_rad: float
