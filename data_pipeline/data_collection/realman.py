from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from data_pipeline.data_collection.config import RealManConfig


FORCE_DRAG_MODE_VALUES = {"fast": 0, "precise": 1}
FORCE_DRAG_MODE_NAMES = {0: "fast", 1: "precise"}
DRAG_MODE_NAMES = {
    0: "电流环",
    1: "末端六维力-只动位置",
    2: "末端六维力-只动姿态",
    3: "末端六维力-位置和姿态",
}


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
        self._replay_active = False

    @property
    def is_connected(self) -> bool:
        return self._arm is not None

    @property
    def is_drag_enabled(self) -> bool:
        """返回拖动示教是否处于已开启状态。"""
        return self._drag_enabled

    @property
    def is_replay_active(self) -> bool:
        """返回控制器拖动轨迹是否正在复现。"""
        return self._replay_active

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
        expected_force_drag_mode: int | None = None
        if self.config.drag_mode != 0:
            expected_force_drag_mode = FORCE_DRAG_MODE_VALUES[
                self.config.force_drag_mode
            ]
            set_result = arm.rm_set_force_drag_mode(expected_force_drag_mode)
            if set_result != 0:
                raise RuntimeError(
                    "rm_set_force_drag_mode 失败，"
                    f"requested={self.config.force_drag_mode}"
                    f"({expected_force_drag_mode})，SDK code={set_result}"
                )

        result = arm.rm_start_multi_drag_teach(
            mode=self.config.drag_mode,
            singular_wall=int(self.config.singular_wall),
        )
        if result != 0:
            raise RuntimeError(f"rm_start_multi_drag_teach 失败，SDK code={result}")
        self._drag_enabled = True

        if expected_force_drag_mode is not None:
            try:
                mode_result = arm.rm_get_force_drag_mode()
                if (
                    not isinstance(mode_result, tuple)
                    or len(mode_result) < 2
                    or mode_result[0] != 0
                ):
                    code = (
                        mode_result[0]
                        if isinstance(mode_result, tuple) and mode_result
                        else "unknown"
                    )
                    raise RuntimeError(
                        f"rm_get_force_drag_mode 失败，SDK code={code}"
                    )
                actual_force_drag_mode = int(mode_result[1])
                if actual_force_drag_mode not in FORCE_DRAG_MODE_NAMES:
                    raise RuntimeError(
                        "rm_get_force_drag_mode 返回未知模式，"
                        f"actual={actual_force_drag_mode}"
                    )
                if actual_force_drag_mode != expected_force_drag_mode:
                    raise RuntimeError(
                        "六维力拖动模式回读不一致，"
                        f"requested={self.config.force_drag_mode}"
                        f"({expected_force_drag_mode})，"
                        f"actual={FORCE_DRAG_MODE_NAMES[actual_force_drag_mode]}"
                        f"({actual_force_drag_mode})"
                    )
            except Exception as exc:
                stop_result = arm.rm_stop_drag_teach()
                if stop_result != 0:
                    raise RuntimeError(
                        f"{exc}；回读失败后停止拖动示教也失败，"
                        f"SDK code={stop_result}"
                    ) from exc
                self._drag_enabled = False
                raise

            print(
                "六维力拖动模式回读成功: "
                f"requested={self.config.force_drag_mode}"
                f"({expected_force_drag_mode}); "
                f"actual={FORCE_DRAG_MODE_NAMES[actual_force_drag_mode]}"
                f"({actual_force_drag_mode})",
                flush=True,
            )

        force_drag_status = (
            f"{self.config.force_drag_mode}({expected_force_drag_mode})"
            if expected_force_drag_mode is not None
            else "not_applicable"
        )
        singular_wall_status = (
            "enabled" if self.config.singular_wall else "disabled"
        )
        if self.config.drag_mode == 0:
            singular_wall_status = "not_applicable"
        print(
            "拖动示教状态: 已开启; "
            f"drag_mode={self.config.drag_mode}"
            f"({DRAG_MODE_NAMES[self.config.drag_mode]}); "
            f"force_drag_mode={force_drag_status}; "
            f"singular_wall={singular_wall_status}; SDK code=0",
            flush=True,
        )

    def stop_drag_teach(self) -> None:
        if self._arm is None or not self._drag_enabled:
            return
        result = self._arm.rm_stop_drag_teach()
        if result != 0:
            raise RuntimeError(f"rm_stop_drag_teach 失败，SDK code={result}")
        self._drag_enabled = False
        print("拖动示教状态: 已结束; SDK code=0", flush=True)

    def save_drag_trajectory(self, output_path: Path) -> int:
        """保存控制器中最近一次拖动示教轨迹并返回点数。"""
        arm = self._require_arm()
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝保存轨迹")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝保存轨迹")
        if output_path.exists():
            raise FileExistsError(f"轨迹输出路径已存在: {output_path}")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_path = str(output_path.resolve())
        if len(resolved_path) > 300:
            raise ValueError("RealMan 轨迹输出绝对路径长度超过 300 个字符")

        result = arm.rm_save_trajectory(resolved_path)
        if not isinstance(result, tuple) or len(result) < 2 or result[0] != 0:
            code = result[0] if isinstance(result, tuple) and result else "unknown"
            raise RuntimeError(f"rm_save_trajectory 失败，SDK code={code}")
        point_count = int(result[1])
        if point_count < 2:
            raise RuntimeError(
                "控制器没有生成可复现的拖动轨迹，"
                f"trajectory_point_count={point_count}"
            )
        if not output_path.is_file():
            raise RuntimeError(f"SDK 返回成功但轨迹文件不存在: {output_path}")
        print(
            "拖动示教轨迹已保存: "
            f"path={output_path}; point_count={point_count}; SDK code=0",
            flush=True,
        )
        return point_count

    def move_to_drag_trajectory_origin(self) -> None:
        """阻塞运动到最近一次拖动示教轨迹的起点。"""
        arm = self._require_arm()
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝运动到轨迹起点")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝重复运动到轨迹起点")
        print("机械臂状态: 正在运动到拖动示教轨迹起点，不录制 rosbag2", flush=True)
        result = arm.rm_drag_trajectory_origin(1)
        if result != 0:
            raise RuntimeError(f"rm_drag_trajectory_origin 失败，SDK code={result}")
        print("机械臂状态: 已到达拖动示教轨迹起点; SDK code=0", flush=True)

    def run_drag_trajectory(self) -> None:
        """阻塞复现控制器中最近一次拖动示教轨迹。"""
        arm = self._require_arm()
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝复现轨迹")
        if self._replay_active:
            raise RuntimeError("拖动示教轨迹已经在复现")
        self._replay_active = True
        print("轨迹复现状态: 正在运行", flush=True)
        try:
            result = arm.rm_run_drag_trajectory(1)
            if result != 0:
                raise RuntimeError(f"rm_run_drag_trajectory 失败，SDK code={result}")
        finally:
            self._replay_active = False
        print("轨迹复现状态: 已完成; SDK code=0", flush=True)

    def stop_drag_trajectory(self) -> None:
        """停止当前正在运行的拖动示教轨迹复现。"""
        if self._arm is None or not self._replay_active:
            return
        result = self._arm.rm_stop_drag_trajectory()
        if result != 0:
            raise RuntimeError(f"rm_stop_drag_trajectory 失败，SDK code={result}")
        deadline = time.monotonic() + 10.0
        while self._replay_active and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._replay_active:
            raise RuntimeError("停止轨迹复现指令成功，但阻塞回放未在 10 秒内结束")
        print("轨迹复现状态: 已停止; SDK code=0", flush=True)

    def restore_home(self) -> None:
        arm = self._require_arm()
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝执行返航")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝执行返航")
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
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝断开 RealMan SDK")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝断开 RealMan SDK")
        arm = self._arm
        self._arm = None
        self._home_joints_deg = None
        self._drag_enabled = False
        self._replay_active = False
        result = arm.rm_delete_robot_arm()
        if result not in (None, 0):
            raise RuntimeError(f"rm_delete_robot_arm 失败，SDK code={result}")
