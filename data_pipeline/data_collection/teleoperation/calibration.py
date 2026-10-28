"""Two-reference-pose calibration inspired by JoyLo; no controller tuning here."""
from __future__ import annotations

import numpy as np


def calibrate_two_poses(device, zero_raw, reference_raw, zero_joints_deg, reference_joints_deg):
    c = device
    zero_raw, reference_raw = np.asarray(zero_raw, dtype=float), np.asarray(reference_raw, dtype=float)
    expected_zero, expected_reference = np.asarray(zero_joints_deg, dtype=float), np.asarray(reference_joints_deg, dtype=float)
    if zero_raw.shape != (len(c.servo_ids),) or reference_raw.shape != zero_raw.shape or expected_zero.shape != (6,) or expected_reference.shape != (6,):
        raise ValueError('标定读数或参考姿态维度错误')
    if not all(np.isfinite(v).all() for v in (zero_raw, reference_raw, expected_zero, expected_reference)):
        raise ValueError('标定数据包含 NaN/Inf')
    if np.any(zero_raw < c.raw_min) or np.any(zero_raw > c.raw_max) or np.any(reference_raw < c.raw_min) or np.any(reference_raw > c.raw_max):
        raise ValueError('标定读数超出设备反馈量程')
    degrees_per_raw = np.asarray(c.angle_range_deg)/(np.asarray(c.raw_max)-c.raw_min)
    measured_delta = ((reference_raw-zero_raw)*degrees_per_raw)[list(c.joint_indices)]
    expected_delta = expected_reference-expected_zero
    if np.any(np.abs(expected_delta) < 5) or np.any(np.abs(measured_delta) < 5):
        raise ValueError('两个姿态中每个映射关节都需要至少 5° 变化，不能判断未移动关节的方向')
    signs = np.sign(measured_delta/expected_delta)
    # Preserve the device's specified encoder scale; do not fit a bad pose by rescaling.
    mismatch = np.abs(signs*measured_delta-expected_delta)
    if np.any(mismatch > 10):
        raise ValueError(f'两姿态角度差不一致，检查量程、对应关节和摆放姿态: {mismatch.round(2).tolist()}°')
    allowed = ('model','servo_ids','raw_min','raw_max','angle_range_deg','joint_indices')
    result = {key:c.document[key] for key in allowed if key in c.document}
    result.update(version=2, verified=True, zero_raw=zero_raw.tolist(),
                  zero_joints_deg=expected_zero.tolist(), direction=signs.astype(int).tolist(),
                  scale=[1.0]*6, home_raw=zero_raw.round().astype(int).tolist(),
                  calibration={
                      'method':'two_reference_poses', 'reference_raw':reference_raw.tolist(),
                      'reference_joints_deg':expected_reference.tolist(), 'residual_deg':mismatch.tolist(),
                      'reference_source':'IUNONE/BEHAVIOR-1K joylo/scripts/calibrate_joints.py',
                  })
    return result
