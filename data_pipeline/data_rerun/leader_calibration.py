"""Persistent encoder-to-U-Arm model calibration, independent of follower mapping."""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

import numpy as np

DEVICE_FIELDS = ('servo_ids', 'raw_min', 'raw_max', 'angle_range_deg', 'joint_indices')


def joint_direction(calibration, index, zero, positive):
    channel = calibration.joint_indices[index]
    lo, hi = calibration.raw_min[channel], calibration.raw_max[channel]
    if not all(np.isfinite(v) and lo <= v <= hi for v in (zero, positive)):
        raise ValueError('编码器读数超出配置范围')
    delta_deg = (positive - zero) / (hi - lo) * calibration.angle_range_deg[channel]
    if not 5 <= abs(delta_deg) <= 90:
        raise ValueError('请从刚记录的零位沿模型正方向转动 5°～90°，避免跨越编码器回绕点')
    return 1 if delta_deg > 0 else -1


class LeaderModelCalibration:
    def __init__(self, document, calibration, urdf):
        if document.get('kind') != 'uarm_model_calibration' or document.get('version') != 1:
            raise ValueError('不是受支持的主臂模型标定文件')
        if document.get('device') != {key: list(getattr(calibration, key)) for key in DEVICE_FIELDS}:
            raise ValueError('主臂模型标定与当前编码器配置或 joint_indices 不匹配')
        if document.get('urdf_sha256') != hashlib.sha256(Path(urdf).read_bytes()).hexdigest():
            raise ValueError('URDF 已改变，请重新进行主臂模型标定')
        zero, positive = document.get('zero_raw', []), document.get('positive_raw', [])
        if len(zero) != 6 or len(positive) != 6:
            raise ValueError('主臂模型标定需要六个关节的零位和正方向读数')
        directions = [joint_direction(calibration, i, z, p) for i, (z, p) in enumerate(zip(zero, positive))]
        if document.get('direction') != directions:
            raise ValueError('正方向标记与记录的编码器读数不一致')
        self.document = document
        self.indices = list(calibration.joint_indices)
        self.direction = np.asarray(directions)
        indices = self.indices
        self.zero_angles = np.deg2rad(
            (np.asarray(zero) - np.asarray(calibration.raw_min)[indices])
            / (np.asarray(calibration.raw_max)[indices] - np.asarray(calibration.raw_min)[indices])
            * np.asarray(calibration.angle_range_deg)[indices]
        )

    @classmethod
    def from_readings(cls, calibration, urdf, zero, positive):
        document = {
            'kind': 'uarm_model_calibration', 'version': 1,
            'device': {key: list(getattr(calibration, key)) for key in DEVICE_FIELDS},
            'urdf_sha256': hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
            # These arrays are in joint order, not servo channel order.
            'zero_raw': list(zero), 'positive_raw': list(positive),
            'direction': [joint_direction(calibration, i, z, p) for i, (z, p) in enumerate(zip(zero, positive))],
        }
        return cls(document, calibration, urdf)

    @classmethod
    def load(cls, path, calibration, urdf):
        return cls(json.loads(Path(path).read_text()), calibration, urdf)

    def joint_positions(self, positions):
        return self.direction * (np.asarray(positions)[self.indices] - self.zero_angles)


def sample_stable_raw(read, channels, servo_ids, poll_interval, *, duration=0.8, timeout=3.0):
    """Require fresh scans, checking only the channels being calibrated."""
    channels = list(channels)
    started = time.monotonic()
    # Do not count the cached scan that preceded the confirmation key.
    baseline = read().timestamp_ns
    seen, samples = set(), []
    while time.monotonic() - started < timeout:
        sample = read()
        if sample.timestamp_ns > baseline and sample.timestamp_ns not in seen:
            seen.add(sample.timestamp_ns)
            samples.append(sample.raw)
        if len(samples) >= 3 and time.monotonic() - started >= duration:
            break
        time.sleep(poll_interval)
    if len(samples) < 3:
        raise ValueError(f'新样本不足：{timeout:g} 秒内仅收到 {len(samples)} 个新扫描（至少需要 3 个）；请检查串口读取频率')
    values = np.asarray(samples)
    spread = np.ptp(values[:, channels], axis=0)
    unstable = [f'通道 {channel} / 舵机 {servo_ids[channel]} 波动 {change:g} raw'
                for channel, change in zip(channels, spread) if change > 10]
    if unstable:
        raise ValueError('当前标定关节读数不稳定：' + '；'.join(unstable) + '（允许最多 10 raw）')
    return np.median(values, axis=0)
