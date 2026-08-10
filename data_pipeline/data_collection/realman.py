from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from data_pipeline.data_collection.config import RealManConfig


@dataclass(frozen=True, slots=True)
class ArmObservation:
    """One direct-SDK arm sample in the canonical project units."""

    timestamp_ns: int
    joint_position: np.ndarray
    ee_pose: np.ndarray
    force: np.ndarray


def euler_xyz_to_quaternion_wxyz(euler_rad: np.ndarray) -> np.ndarray:
    """Convert intrinsic XYZ Euler angles to a wxyz quaternion without SciPy."""
    roll, pitch, yaw = (float(value) for value in euler_rad)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float32,
    )


class RealManArm:
    """Thin lifecycle adapter over the official ``Robotic_Arm`` Python SDK."""

    def __init__(self, config: RealManConfig) -> None:
        config.validate()
        self.config = config
        self._arm: Any | None = None
        self._home_joints_deg: np.ndarray | None = None
        self._drag_enabled = False

    @property
    def is_connected(self) -> bool:
        return self._arm is not None

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("RealMan SDK 已连接")
        try:
            from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
        except ImportError as exc:
            raise ImportError(
                "缺少 RealMan 官方 Python SDK；请在当前环境安装 Robotic_Arm"
            ) from exc

        arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        arm.rm_create_robot_arm(self.config.ip, self.config.port)
        result = arm.rm_get_current_arm_state()
        if not isinstance(result, tuple) or len(result) < 2 or result[0] != 0:
            try:
                arm.rm_delete_robot_arm()
            finally:
                code = result[0] if isinstance(result, tuple) and result else "unknown"
            raise ConnectionError(f"连接 RealMan 失败，SDK code={code}")

        joints = np.asarray(result[1]["joint"][:6], dtype=np.float32)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            arm.rm_delete_robot_arm()
            raise RuntimeError("机械臂初始关节状态无效")
        self._arm = arm
        self._home_joints_deg = joints.copy()

    def _require_arm(self) -> Any:
        if self._arm is None:
            raise RuntimeError("RealMan SDK 未连接")
        return self._arm

    def read_observation(self) -> ArmObservation:
        arm = self._require_arm()
        state_result = arm.rm_get_current_arm_state()
        if state_result[0] != 0:
            raise RuntimeError(f"rm_get_current_arm_state 失败，SDK code={state_result[0]}")
        state = state_result[1]
        joints_deg = np.asarray(state["joint"][:6], dtype=np.float32)
        pose = np.asarray(state["pose"][:6], dtype=np.float32)
        if joints_deg.shape != (6,) or pose.shape != (6,):
            raise RuntimeError("RealMan SDK 返回了非 ECO63 六轴状态")
        if not np.isfinite(joints_deg).all() or not np.isfinite(pose).all():
            raise RuntimeError("RealMan SDK 返回了 NaN/Inf 状态")

        force_result = arm.rm_get_force_data()
        if force_result[0] != 0:
            raise RuntimeError(f"rm_get_force_data 失败，SDK code={force_result[0]}")
        force = np.asarray(list(force_result[1]["zero_force_data"])[:6], dtype=np.float32)
        if force.shape != (6,) or not np.isfinite(force).all():
            raise RuntimeError("RealMan SDK 返回了无效六维力数据")

        quaternion = euler_xyz_to_quaternion_wxyz(pose[3:6])
        return ArmObservation(
            timestamp_ns=time.monotonic_ns(),
            joint_position=np.deg2rad(joints_deg).astype(np.float32),
            ee_pose=np.concatenate((pose[:3], quaternion)).astype(np.float32),
            force=force,
        )

    def enable_drag_teach(self) -> None:
        arm = self._require_arm()
        result = arm.rm_start_multi_drag_teach(
            mode=self.config.drag_mode,
            singular_wall=int(self.config.singular_wall),
        )
        if result != 0:
            raise RuntimeError(f"rm_start_multi_drag_teach 失败，SDK code={result}")
        self._drag_enabled = True

    def stop_drag_teach(self) -> None:
        if self._arm is None or not self._drag_enabled:
            return
        result = self._arm.rm_stop_drag_teach()
        if result != 0:
            raise RuntimeError(f"rm_stop_drag_teach 失败，SDK code={result}")
        self._drag_enabled = False

    def restore_home(self) -> None:
        arm = self._require_arm()
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝执行返航")
        if self._home_joints_deg is None:
            raise RuntimeError("没有可用的初始关节位姿")

        preflight = arm.rm_get_current_arm_state()
        if preflight[0] != 0:
            raise RuntimeError("返航前无法确认机械臂状态，拒绝运动")
        result = arm.rm_movej(
            self._home_joints_deg.tolist(),
            v=self.config.home_return_speed,
            r=0,
            connect=0,
            block=1,
        )
        if result != 0:
            raise RuntimeError(f"rm_movej 返航失败，SDK code={result}")
        final_state = arm.rm_get_current_arm_state()
        if final_state[0] != 0:
            raise RuntimeError("返航后无法读取机械臂状态")
        final_joints = np.asarray(final_state[1]["joint"][:6], dtype=np.float32)
        max_error = float(np.max(np.abs(final_joints - self._home_joints_deg)))
        if max_error > self.config.home_tolerance_deg:
            raise RuntimeError(
                f"返航关节误差 {max_error:.3f} deg 超过 "
                f"{self.config.home_tolerance_deg:.3f} deg"
            )

    def disconnect(self) -> None:
        if self._arm is None:
            return
        arm = self._arm
        self._arm = None
        self._home_joints_deg = None
        self._drag_enabled = False
        result = arm.rm_delete_robot_arm()
        if result not in (None, 0):
            raise RuntimeError(f"rm_delete_robot_arm 失败，SDK code={result}")
