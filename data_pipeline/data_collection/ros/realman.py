from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from data_pipeline.data_collection.config import RealManConfig
from data_pipeline.data_collection.drag.config import (
    DRAG_MODE_NAMES,
    FORCE_DRAG_MODE_NAMES,
    FORCE_DRAG_MODE_VALUES,
)


@dataclass(frozen=True, slots=True)
class RosArmObservation:
    """保存 ROS driver 发布的六关节位置。"""

    joint_position: np.ndarray


class RealManRosArm:
    """通过官方 ROS driver 的单一 SDK handle 控制机械臂。"""

    def __init__(
        self,
        config: RealManConfig,
        *,
        startup_timeout_s: float,
        sample_timeout_s: float,
    ) -> None:
        """保存连接参数并初始化本地 ROS 控制状态。"""
        config.validate()
        self.config = config
        self.startup_timeout_s = startup_timeout_s
        self.sample_timeout_s = sample_timeout_s
        self._context: Any | None = None
        self._node: Any | None = None
        self._executor: Any | None = None
        self._spin_thread: threading.Thread | None = None
        self._clients: dict[str, Any] = {}
        self._service_types: dict[str, Any] = {}
        self._subscriptions: list[Any] = []
        self._movej_publisher: Any | None = None
        self._movej_type: Any | None = None
        self._condition = threading.Condition()
        self._latest_joint_position: np.ndarray | None = None
        self._latest_joint_received_at: float | None = None
        self._joint_sequence = 0
        self._last_read_joint_sequence = 0
        self._movej_result_sequence = 0
        self._movej_result = False
        self._home_joints_rad: np.ndarray | None = None
        self._drag_enabled = False
        self._replay_active = False

    @property
    def is_connected(self) -> bool:
        """返回 ROS 控制节点是否已连接。"""
        return self._node is not None

    @property
    def is_drag_enabled(self) -> bool:
        """返回拖动示教是否处于开启状态。"""
        return self._drag_enabled

    @property
    def is_replay_active(self) -> bool:
        """返回拖动轨迹是否正在复现。"""
        return self._replay_active

    def _joint_state_callback(self, message: Any) -> None:
        """缓存最新的有效六关节位置。"""
        joints = np.asarray(message.position[:6], dtype=np.float32)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            return
        with self._condition:
            self._latest_joint_position = joints.copy()
            self._latest_joint_received_at = time.monotonic()
            self._joint_sequence += 1
            self._condition.notify_all()

    def _movej_result_callback(self, message: Any) -> None:
        """缓存最近一次 MoveJ 指令的执行结果。"""
        with self._condition:
            self._movej_result = bool(message.data)
            self._movej_result_sequence += 1
            self._condition.notify_all()

    def _shutdown_ros(self) -> None:
        """关闭本进程的 ROS 客户端资源。"""
        executor = self._executor
        node = self._node
        context = self._context
        self._executor = None
        self._node = None
        self._context = None
        self._clients.clear()
        self._service_types.clear()
        self._subscriptions.clear()
        self._movej_publisher = None
        self._movej_type = None
        if executor is not None:
            executor.shutdown(timeout_sec=5.0)
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=5.0)
            self._spin_thread = None
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()

    def connect(self, *, enable_drag_services: bool = True) -> None:
        """Connect joint topics; drag services are optional for teleoperation."""
        if self.is_connected:
            raise RuntimeError("RealMan ROS 控制节点已连接")
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from rm_ros_interfaces.msg import Movej
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool
        except ImportError as exc:
            raise ImportError(
                "缺少 ROS 关节控制消息接口，请确认已加载工作区 install/setup 环境"
            ) from exc

        try:
            context = Context()
            rclpy.init(context=context)
            node = Node("xiazhi_realman_control", context=context)
            executor = MultiThreadedExecutor(num_threads=4, context=context)
            executor.add_node(node)
            self._context = context
            self._node = node
            self._executor = executor
            self._service_types = {}
            self._clients = {}
            if enable_drag_services:
                try:
                    from rm_ros_interfaces.srv import (
                        CommandResult, SaveDragTrajectory, SetForceDragMode, StartMultiDragTeach,
                    )
                except ImportError as exc:
                    raise ImportError('拖动示教需要扩展 ROS service 接口，请重新编译 rm_ros_interfaces 和 rm_driver') from exc
                self._service_types = {
                    "command": CommandResult,
                    "save": SaveDragTrajectory,
                    "set_force": SetForceDragMode,
                    "start_drag": StartMultiDragTeach,
                }
                self._clients = {
                    "set_force": node.create_client(
                        SetForceDragMode,
                        "/rm_driver/set_force_drag_mode",
                    ),
                    "start_drag": node.create_client(
                        StartMultiDragTeach,
                        "/rm_driver/start_multi_drag_teach",
                    ),
                    "stop_drag": node.create_client(
                        CommandResult,
                        "/rm_driver/stop_drag_teach",
                    ),
                    "save": node.create_client(
                        SaveDragTrajectory,
                        "/rm_driver/save_drag_trajectory",
                    ),
                    "origin": node.create_client(
                        CommandResult,
                        "/rm_driver/drag_trajectory_origin",
                    ),
                    "run": node.create_client(
                        CommandResult,
                        "/rm_driver/run_drag_trajectory",
                    ),
                    "stop_replay": node.create_client(
                        CommandResult,
                        "/rm_driver/stop_drag_trajectory",
                    ),
                }
            self._subscriptions = [
                node.create_subscription(
                    JointState,
                    "/joint_states",
                    self._joint_state_callback,
                    10,
                ),
                node.create_subscription(
                    Bool,
                    "/rm_driver/movej_result",
                    self._movej_result_callback,
                    10,
                ),
            ]
            self._movej_publisher = node.create_publisher(
                Movej,
                "/rm_driver/movej_cmd",
                10,
            )
            self._movej_type = Movej
            self._spin_thread = threading.Thread(
                target=executor.spin,
                name="xiazhi-realman-ros",
                daemon=True,
            )
            self._spin_thread.start()

            deadline = time.monotonic() + self.startup_timeout_s
            for service_name, client in self._clients.items():
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
                    raise TimeoutError(f"等待 RealMan ROS service 超时: {service_name}")
            while self._movej_publisher.get_subscription_count() == 0:
                if time.monotonic() >= deadline:
                    raise TimeoutError("等待 /rm_driver/movej_cmd subscriber 超时")
                time.sleep(0.05)

            initial = self.read_observation().joint_position
            if self.config.init_joints_deg is None:
                self._home_joints_rad = initial.copy()
            else:
                self._home_joints_rad = np.deg2rad(
                    np.asarray(self.config.init_joints_deg, dtype=np.float32)
                )
            print("RealMan 控制连接: 复用 rm_driver 单一 SDK handle", flush=True)
        except BaseException:
            self._shutdown_ros()
            raise

    def _call_service(
        self,
        client_name: str,
        request: Any,
        operation: str,
        *,
        timeout_s: float | None = None,
    ) -> Any:
        """调用一个 RealMan ROS service 并返回响应。"""
        if not self.is_connected:
            raise RuntimeError("RealMan ROS 控制节点未连接")
        future = self._clients[client_name].call_async(request)
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while not future.done():
            if not self.is_connected:
                raise RuntimeError(f"{operation} 等待期间 ROS 控制节点已断开")
            if deadline is not None and time.monotonic() >= deadline:
                future.cancel()
                raise TimeoutError(f"{operation} 超时")
            time.sleep(0.01)
        error = future.exception()
        if error is not None:
            raise RuntimeError(f"{operation} service 调用失败: {error}") from error
        response = future.result()
        if response is None:
            raise RuntimeError(f"{operation} service 没有返回响应")
        return response

    @staticmethod
    def _check_status(response: Any, operation: str) -> None:
        """检查 RealMan service 返回的 SDK 状态码。"""
        if response.status != 0:
            raise RuntimeError(f"{operation} 失败，SDK code={response.status}")

    def read_observation(self) -> RosArmObservation:
        """等待并返回一帧新的六关节 ROS 状态。"""
        if not self.is_connected:
            raise RuntimeError("RealMan ROS 控制节点未连接")
        deadline = time.monotonic() + self.sample_timeout_s
        with self._condition:
            while (
                self._latest_joint_position is None
                or self._joint_sequence <= self._last_read_joint_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("等待新的 /joint_states 消息超时")
                self._condition.wait(timeout=remaining)
            joints = self._latest_joint_position.copy()
            self._last_read_joint_sequence = self._joint_sequence
        return RosArmObservation(joint_position=joints)

    def latest_observation(self) -> RosArmObservation:
        """Read cached feedback without blocking; reject stale/disconnected data."""
        if not self.is_connected:
            raise RuntimeError('RealMan ROS 控制节点未连接')
        with self._condition:
            if (self._latest_joint_position is None or self._latest_joint_received_at is None
                    or time.monotonic() - self._latest_joint_received_at > self.sample_timeout_s):
                raise TimeoutError('从臂 /joint_states 反馈过期，停止跟随')
            return RosArmObservation(joint_position=self._latest_joint_position.copy())

    def prepare_teleoperation(self) -> None:
        self.sample_timeout_s = min(self.sample_timeout_s, 0.2)
        from rm_ros_interfaces.msg import Jointpos
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Empty, Bool
        from data_pipeline.data_collection.teleoperation.config import COMMAND_TOPIC, LEADER_TOPIC

        self._target_type, self._empty_type, self._joint_type = Jointpos, Empty, JointState
        self._target_pub = self._node.create_publisher(Jointpos, "/rm_driver/movej_canfd_cmd", 1)
        self._stop_pub = self._node.create_publisher(Empty, "/rm_driver/move_stop_cmd", 1)
        self._command_pub = self._node.create_publisher(JointState, COMMAND_TOPIC, 100)
        self._leader_pub = self._node.create_publisher(JointState, LEADER_TOPIC, 100)
        self._stop_sequence = 0
        self._stop_result = False
        self._subscriptions.append(self._node.create_subscription(Bool, "/rm_driver/move_stop_result", self._stop_callback, 10))
        deadline = time.monotonic() + self.startup_timeout_s
        while not self._target_pub.get_subscription_count() or not self._stop_pub.get_subscription_count():
            if time.monotonic() >= deadline:
                raise TimeoutError("等待 RealMan 透传/停止订阅超时")
            time.sleep(0.05)

    def _stop_callback(self, message) -> None:
        with self._condition:
            self._stop_sequence += 1
            self._stop_result = bool(message.data)
            self._condition.notify_all()

    def send_joint_target(self, target: np.ndarray) -> None:
        if self._drag_enabled or self._replay_active:
            raise RuntimeError("拖动或回放期间禁止透传")
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError("目标必须为六个有限弧度值")
        if not self._target_pub.get_subscription_count():
            raise RuntimeError("RealMan 透传订阅已断开")
        message = self._target_type()
        message.joint, message.follow, message.expand, message.dof = target.tolist(), False, 0.0, 6
        self._target_pub.publish(message)

    def publish_teleoperation(self, command) -> None:
        # Driver does not acknowledge individual CANFD commands: this is submitted,
        # not confirmed-executed action. Canonical recorded values stay in radians.
        message = self._joint_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = str(command.sequence)
        message.name = [f"joint_{i}" for i in range(1, 7)]
        message.position = list(command.target)
        self._command_pub.publish(message)
        leader = self._joint_type()
        leader.header.stamp = message.header.stamp
        leader.name = [f"leader_{i}" for i in range(len(command.leader_position))]
        leader.position = list(command.leader_position)
        self._leader_pub.publish(leader)

    def stop_motion(self) -> None:
        with self._condition:
            sequence = self._stop_sequence
            self._stop_pub.publish(self._empty_type())
            deadline = time.monotonic() + 1.0
            while self._stop_sequence == sequence:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("RealMan stop acknowledgment timed out")
                self._condition.wait(remaining)
            if not self._stop_result:
                raise RuntimeError("RealMan stop failed")

    def enable_drag_teach(self) -> None:
        """设置六维力模式并通过 rm_driver 开启拖动示教。"""
        if self._drag_enabled:
            raise RuntimeError("拖动示教已经开启")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝开启拖动示教")
        expected_force_mode: int | None = None
        if self.config.drag_mode != 0:
            expected_force_mode = FORCE_DRAG_MODE_VALUES[self.config.force_drag_mode]
            request = self._service_types["set_force"].Request()
            request.mode = expected_force_mode
            response = self._call_service(
                "set_force",
                request,
                "rm_set_force_drag_mode",
                timeout_s=self.startup_timeout_s,
            )
            self._check_status(response, "rm_set_force_drag_mode")
            if response.actual_mode != expected_force_mode:
                raise RuntimeError(
                    "六维力拖动模式回读不一致，"
                    f"requested={expected_force_mode}, actual={response.actual_mode}"
                )
            print(
                "六维力拖动模式回读成功: "
                f"requested={self.config.force_drag_mode}({expected_force_mode}); "
                f"actual={FORCE_DRAG_MODE_NAMES[response.actual_mode]}"
                f"({response.actual_mode})",
                flush=True,
            )

        request = self._service_types["start_drag"].Request()
        request.mode = self.config.drag_mode
        request.singular_wall = self.config.singular_wall
        response = self._call_service(
            "start_drag",
            request,
            "rm_start_multi_drag_teach",
            timeout_s=self.startup_timeout_s,
        )
        self._check_status(response, "rm_start_multi_drag_teach")
        self._drag_enabled = True
        force_status = (
            f"{self.config.force_drag_mode}({expected_force_mode})"
            if expected_force_mode is not None
            else "not_applicable"
        )
        singular_status = "enabled" if self.config.singular_wall else "disabled"
        if self.config.drag_mode == 0:
            singular_status = "not_applicable"
        print(
            "拖动示教状态: 已开启; "
            f"drag_mode={self.config.drag_mode}"
            f"({DRAG_MODE_NAMES[self.config.drag_mode]}); "
            f"force_drag_mode={force_status}; "
            f"singular_wall={singular_status}; SDK code=0",
            flush=True,
        )

    def stop_drag_teach(self) -> None:
        """通过 rm_driver 停止当前拖动示教。"""
        if not self._drag_enabled:
            return
        request = self._service_types["command"].Request()
        response = self._call_service(
            "stop_drag",
            request,
            "rm_stop_drag_teach",
            timeout_s=self.startup_timeout_s,
        )
        self._check_status(response, "rm_stop_drag_teach")
        self._drag_enabled = False
        print("拖动示教状态: 已结束; SDK code=0", flush=True)

    def save_drag_trajectory(self, output_path: Path) -> int:
        """通过 rm_driver 保存控制器拖动轨迹并返回点数。"""
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
        request = self._service_types["save"].Request()
        request.path = resolved_path
        response = self._call_service(
            "save",
            request,
            "rm_save_trajectory",
        )
        self._check_status(response, "rm_save_trajectory")
        point_count = int(response.point_count)
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
        """通过 rm_driver 阻塞运动到拖动轨迹起点。"""
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝运动到轨迹起点")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝运动到轨迹起点")
        print("机械臂状态: 正在运动到拖动示教轨迹起点，不录制 rosbag2", flush=True)
        request = self._service_types["command"].Request()
        response = self._call_service(
            "origin",
            request,
            "rm_drag_trajectory_origin",
        )
        self._check_status(response, "rm_drag_trajectory_origin")
        print("机械臂状态: 已到达拖动示教轨迹起点; SDK code=0", flush=True)

    def run_drag_trajectory(self) -> None:
        """通过 rm_driver 阻塞复现最近一次拖动轨迹。"""
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝复现轨迹")
        if self._replay_active:
            raise RuntimeError("拖动示教轨迹已经在复现")
        self._replay_active = True
        print("轨迹复现状态: 正在运行", flush=True)
        try:
            request = self._service_types["command"].Request()
            response = self._call_service(
                "run",
                request,
                "rm_run_drag_trajectory",
            )
            self._check_status(response, "rm_run_drag_trajectory")
        finally:
            self._replay_active = False
        print("轨迹复现状态: 已完成; SDK code=0", flush=True)

    def stop_drag_trajectory(self) -> None:
        """通过独立 ROS service 停止阻塞中的轨迹复现。"""
        if not self._replay_active:
            return
        request = self._service_types["command"].Request()
        response = self._call_service(
            "stop_replay",
            request,
            "rm_stop_drag_trajectory",
            timeout_s=10.0,
        )
        self._check_status(response, "rm_stop_drag_trajectory")
        deadline = time.monotonic() + 10.0
        while self._replay_active and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._replay_active:
            raise RuntimeError("停止轨迹复现成功，但阻塞回放未在 10 秒内结束")
        print("轨迹复现状态: 已停止; SDK code=0", flush=True)

    def validate_home(self, lower, upper) -> None:
        if self._home_joints_rad is None:
            raise RuntimeError("未设置从臂初始位置")
        if np.any(self._home_joints_rad < lower) or np.any(self._home_joints_rad > upper):
            raise ValueError("从臂 init 超出 URDF 关节限位，拒绝回位")

    def restore_home(self) -> None:
        """通过官方 MoveJ topic 低速返回配置的初始关节位置。"""
        if self._home_joints_rad is None:
            raise RuntimeError("没有可用的初始关节位姿")
        self.move_to_joint_pose(self._home_joints_rad)

    def move_to_joint_pose(self, joints_rad) -> None:
        """Move to a fixed absolute snapshot without changing the saved home pose."""
        target = np.asarray(joints_rad, dtype=float).copy()
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError("MoveJ 目标必须为六个有限弧度值")
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝执行返航")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝执行返航")
        if self._movej_publisher is None or self._movej_type is None:
            raise RuntimeError("MoveJ ROS publisher 未初始化")
        message = self._movej_type()
        message.joint = target.tolist()
        message.speed = self.config.home_return_speed
        message.block = True
        message.trajectory_connect = 0
        message.dof = 6
        with self._condition:
            result_sequence = self._movej_result_sequence
            self._movej_publisher.publish(message)
            print(f'MoveJ 目标已提交，速度比例 {message.speed}，正在等待到位确认。', flush=True)
            deadline = time.monotonic() + max(60.0, self.startup_timeout_s)
            while self._movej_result_sequence <= result_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("等待 /rm_driver/movej_result 超时")
                self._condition.wait(timeout=remaining)
            if not self._movej_result:
                raise RuntimeError("rm_driver MoveJ 运动失败")
        final_joints = self.read_observation().joint_position
        max_error = math.degrees(
            float(np.max(np.abs(final_joints - target)))
        )
        if max_error > self.config.home_tolerance_deg:
            raise RuntimeError(
                f"MoveJ 关节误差 {max_error:.3f} deg 超过 "
                f"{self.config.home_tolerance_deg:.3f} deg"
            )

    def disconnect(self) -> None:
        """断开本进程 ROS 客户端并保留 driver 的唯一 SDK 连接。"""
        if not self.is_connected:
            return
        if self._drag_enabled:
            raise RuntimeError("拖动示教尚未停止，拒绝断开 ROS 控制节点")
        if self._replay_active:
            raise RuntimeError("轨迹正在复现，拒绝断开 ROS 控制节点")
        self._home_joints_rad = None
        self._shutdown_ros()
