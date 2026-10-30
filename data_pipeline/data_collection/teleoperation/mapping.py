"""Absolute joint mapping between the leader and follower URDF coordinates."""
from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


class JointMapping:
    def __init__(self, document, leader_sha256, follower_sha256):
        if document.get('kind') != 'leader_follower_mapping' or document.get('version') != 1:
            raise ValueError('Not a leader/follower mapping file.')
        if document.get('mode') != 'absolute' or document.get('joint_names') != list(JOINT_NAMES):
            raise ValueError('Mapping must use absolute control and joint1 through joint6.')
        if document.get('leader_urdf_sha256') != leader_sha256 or document.get('follower_urdf_sha256') != follower_sha256:
            raise ValueError('A robot model has changed. Recheck and save the offline mapping.')
        offset = np.asarray(document.get('offset_deg', []), dtype=float)
        direction = np.asarray(document.get('direction', []), dtype=float)
        if offset.shape != (6,) or not np.isfinite(offset).all():
            raise ValueError('Mapping requires six finite offsets in degrees.')
        if direction.shape != (6,) or not np.isin(direction, [-1, 1]).all():
            raise ValueError('Mapping requires six directions, each +1 or -1.')
        self.document = dict(document)
        self.document['offset_deg'] = offset.tolist()
        self.document['direction'] = direction.astype(int).tolist()
        self.offset_deg = offset
        self.direction = direction.astype(int)

    @classmethod
    def create(cls, offset_deg, direction, leader_sha256, follower_sha256):
        return cls({
            'kind': 'leader_follower_mapping', 'version': 1, 'mode': 'absolute',
            'joint_names': list(JOINT_NAMES),
            'leader_urdf_sha256': leader_sha256, 'follower_urdf_sha256': follower_sha256,
            'offset_deg': list(offset_deg), 'direction': list(direction),
        }, leader_sha256, follower_sha256)

    @classmethod
    def load(cls, path, leader_sha256, follower_sha256):
        return cls(json.loads(Path(path).read_text()), leader_sha256, follower_sha256)

    def apply(self, leader_rad):
        angles = np.asarray(leader_rad, dtype=float)
        if angles.shape != (6,) or not np.isfinite(angles).all():
            raise ValueError('Expected six finite leader URDF joint angles.')
        return np.deg2rad(self.offset_deg) + self.direction * angles

    def inverse(self, follower_rad):
        angles = np.asarray(follower_rad, dtype=float)
        if angles.shape != (6,) or not np.isfinite(angles).all():
            raise ValueError('Expected six finite follower URDF joint angles.')
        # Each sign is its own inverse; offsets are in follower coordinates.
        return self.direction * (angles - np.deg2rad(self.offset_deg))


def compose_absolute_calibration(device, model, mapping, *, require_home=True):
    """Compose the two calibrated coordinate transforms for the hardware reader."""
    home = device.document.get('home_raw') or model.document.get('home_raw')
    if home is None and len(device.servo_ids) == 6:
        home = [0] * 6
        for channel, value in zip(device.joint_indices, model.document['zero_raw']):
            home[channel] = int(round(value))
    if home is None and require_home:
        raise ValueError('缺少完整主臂回位读数 home_raw，请重新执行 1.1 标定或使用 --save-init 保存设备配置')
    if home is not None and (not isinstance(home, list) or len(home) != len(device.servo_ids) or any(
            type(v) is not int or not lo <= v <= hi for v, lo, hi in zip(home, device.raw_min, device.raw_max))):
        raise ValueError('主臂 home_raw 无效')
    # Unmapped channels have no effect on joint_positions and are never home targets.
    zero = list(home) if home is not None else list(device.raw_min)
    for channel, value in zip(device.joint_indices, model.document['zero_raw']):
        zero[channel] = value
    direction = (mapping.direction * model.direction).astype(int).tolist()
    scale = model.scale.tolist()
    document = {
        key: list(getattr(device, key))
        for key in ('servo_ids', 'raw_min', 'raw_max', 'angle_range_deg', 'joint_indices')
    }
    document.update(
        version=2, verified=True, mode='absolute',
        zero_raw=zero, zero_joints_deg=mapping.offset_deg.tolist(),
        direction=direction, scale=scale, home_raw=home,
        model_calibration=model.document, urdf_mapping=mapping.document,
    )
    digest = hashlib.sha256(json.dumps(document, sort_keys=True).encode()).hexdigest()
    return replace(device, direction=tuple(direction), scale=tuple(scale), sha256=digest, document=document)


def load_absolute_calibration(config, *, require_home=True):
    from data_pipeline.data_collection.teleoperation.config import Calibration
    from data_pipeline.data_rerun.arm_pose import ArmModel
    from data_pipeline.data_rerun.leader_calibration import LeaderModelCalibration
    from data_pipeline.data_rerun.leader_preview import LEADER_URDF

    if config.model_calibration is None or config.mapping is None:
        raise ValueError('绝对遥操需要主臂模型标定文件和离线主从映射文件')
    device = Calibration.load(config.calibration, require_verified=False)
    leader_model, follower_model = ArmModel.load('leader'), ArmModel.load('follower')
    model = LeaderModelCalibration.load(config.model_calibration, device, LEADER_URDF)
    mapping = JointMapping.load(config.mapping, leader_model.sha256, follower_model.sha256)
    return compose_absolute_calibration(device, model, mapping, require_home=require_home)
