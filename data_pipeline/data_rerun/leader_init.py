"""Plan physical leader initialization for a virtual follower target."""
from __future__ import annotations

from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class LeaderInitPlan:
    follower_deg: tuple[float, ...]
    leader_deg: tuple[float, ...]
    raw_targets: dict[int, int]

    def check_reached(self, sample, tolerance_raw):
        failed = [i for i, target in self.raw_targets.items() if abs(sample.raw[i] - target) > tolerance_raw]
        if failed:
            raise ValueError(f'主臂已偏离 init 目标（通道 {failed}），请退出后重新执行初始化')


def _check_limits(angles, urdf_bytes, label):
    root = ET.fromstring(urdf_bytes)
    for i, value in enumerate(angles, 1):
        joint = root.find(f"joint[@name='joint{i}']")
        if joint is None or joint.find('limit') is None:
            raise ValueError(f'{label} joint_{i} 缺少 URDF 限位')
        limit = joint.find('limit')
        lo, hi = float(limit.get('lower')), float(limit.get('upper'))
        if not np.isfinite([value, lo, hi]).all() or not lo - 1e-10 <= value <= hi + 1e-10:
            raise ValueError(f'{label} joint_{i} 目标 {np.rad2deg(value):.3f}° 超出 URDF 限位')


def plan_follower_init(follower_deg, device, model_calibration, mapping, leader_urdf, follower_urdf):
    target = np.asarray(follower_deg, dtype=float)
    if target.shape != (6,) or not np.isfinite(target).all():
        raise ValueError('从臂 init 必须为六个有限角度，单位 degree')
    follower = np.deg2rad(target)
    _check_limits(follower, follower_urdf, '从臂')
    leader = mapping.inverse(follower)
    _check_limits(leader, leader_urdf, '主臂')
    raw = model_calibration.raw_targets(leader, device)
    # Check the actual encoder-quantized target as well as the requested pose.
    encoder = np.zeros(len(device.servo_ids))
    for channel, value in zip(model_calibration.indices, raw):
        encoder[channel] = np.deg2rad((value - device.raw_min[channel]) /
                                     (device.raw_max[channel] - device.raw_min[channel]) * device.angle_range_deg[channel])
    _check_limits(model_calibration.joint_positions(encoder), leader_urdf, '主臂量化目标')
    return LeaderInitPlan(tuple(target.tolist()), tuple(np.rad2deg(leader).tolist()),
                          dict(zip(model_calibration.indices, raw)))
