from __future__ import annotations

import argparse
import math
import uuid

from data_pipeline.data_collection.teleoperation.config import LeaderConfig
from pathlib import Path

from data_pipeline.data_collection import (
    CameraConfig,
    CollectionConfig,
    RealManConfig,
    Rosbag2Collector,
)


def parse_init_joints(value: str) -> tuple[float, ...]:
    """解析包含六个角度值的初始关节参数。"""
    normalized = value.strip()
    if normalized.startswith("[") and normalized.endswith("]"):
        normalized = normalized[1:-1]
    parts = [part.strip() for part in normalized.split(",")]
    if len(parts) != 6 or any(not part for part in parts):
        raise argparse.ArgumentTypeError("init_joints 必须包含 6 个逗号分隔的角度")
    try:
        joints = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("init_joints 包含非数值内容") from exc
    if not all(math.isfinite(joint) for joint in joints):
        raise argparse.ArgumentTypeError("init_joints 必须全部是有限数值")
    return joints


def main() -> None:
    """解析 ROS 2 采集参数并执行一个 session。"""
    parser = argparse.ArgumentParser(
        description="Record an ECO63 multi-episode session with ROS 2 rosbag2"
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--session-id", default=None)
    parser.add_argument(
        "--camera-serial",
        action="append",
        required=True,
        help="至少提供一次，可重复提供；按顺序映射为 camera_01、camera_02 等",
    )
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument(
        "--init-joints",
        "--init_joints",
        dest="init_joints",
        type=parse_init_joints,
        default=None,
        help="从臂回位角度；主从模式省略时按 i 对齐当前主臂。zsh 示例 '[0,60,-135,75,90,0]'",
    )
    parser.add_argument("--drag-mode", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument(
        "--force-drag-mode",
        choices=["fast", "precise"],
        default="fast",
        help="六维力拖动轻重模式，仅在 --drag-mode 1、2、3 时生效",
    )
    parser.add_argument("--disable-singular-wall", action="store_true")
    parser.add_argument("--home-return-speed", type=int, default=10)
    parser.add_argument("--home-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--extra-topic", action="append", default=[])
    parser.add_argument("--storage-id", choices=["sqlite3", "mcap"], default="mcap")
    parser.add_argument("--startup-timeout-s", type=float, default=45.0)
    parser.add_argument("--sample-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--capture-mode",
        choices=["direct", "teach-replay"],
        default="direct",
        help="direct 边拖动边录包，teach-replay 先示教再回放录包",
    )
    parser.add_argument("--replay-pre-roll-s", type=float, default=1.0)
    parser.add_argument("--replay-post-roll-s", type=float, default=0.5)
    parser.add_argument("--teaching-mode", choices=["drag", "leader-follower"], default="drag")
    parser.add_argument("--viewer", action="store_true", help="轻量从臂目标预览：i/r 关闭，s 后重开；仅 leader-follower")
    parser.add_argument("--leader-port")
    parser.add_argument("--leader-config", type=Path, help="主臂编码器设备配置")
    parser.add_argument("--leader-model-calibration", type=Path, help="1.1 主臂实物到 URDF 的标定")
    parser.add_argument("--leader-follower-mapping", type=Path, help="1.2 离线主从绝对映射")
    parser.add_argument("--teleop-config", type=Path, help="独立运行配置；默认 configs/teleoperation/eco63_runtime.json")
    parser.add_argument("--leader-baudrate", type=int, default=115200)
    parser.add_argument("--control-hz", type=float, default=30)
    parser.add_argument("--leader-stale-after-s", type=float, default=0.3)
    parser.add_argument("--unlock-leader", action="store_true", help="兼容参数；采集始终在扶稳后按 r 时卸力，连接时不卸力")
    args = parser.parse_args()
    if args.viewer and args.teaching_mode != "leader-follower":
        parser.error("--viewer 需要 --teaching-mode leader-follower")
    leader = None
    if args.teaching_mode == "leader-follower":
        if not args.leader_port or any(value is None for value in (args.leader_config, args.leader_model_calibration, args.leader_follower_mapping)):
            parser.error("leader-follower 需要 --leader-port、--leader-config、--leader-model-calibration 和 --leader-follower-mapping")
        leader = LeaderConfig(args.leader_port, args.leader_config, args.leader_baudrate,
                              args.control_hz, args.leader_stale_after_s, args.teleop_config,
                              args.leader_model_calibration, args.leader_follower_mapping)
    elif args.leader_port or args.leader_config or args.leader_model_calibration or args.leader_follower_mapping or args.unlock_leader or args.teleop_config:
        parser.error("主臂参数需要 --teaching-mode leader-follower")

    cameras = tuple(
        CameraConfig(
            name=f"camera_{index:02d}",
            serial_number=serial,
            fps=args.camera_fps,
            width=args.camera_width,
            height=args.camera_height,
            enable_depth=not args.no_depth,
        )
        for index, serial in enumerate(args.camera_serial, start=1)
    )
    robot = RealManConfig(
        ip=args.robot_ip,
        port=args.robot_port,
        init_joints_deg=args.init_joints,
        drag_mode=args.drag_mode,
        force_drag_mode=args.force_drag_mode,
        singular_wall=not args.disable_singular_wall,
        home_return_speed=args.home_return_speed,
        home_tolerance_deg=args.home_tolerance_deg,
    )
    config = CollectionConfig(
        output_root=args.output_root,
        task=args.task,
        scene=args.scene,
        session_id=args.session_id or str(uuid.uuid4()),
        cameras=cameras,
        robot=robot,
        teaching_mode=args.teaching_mode,
        leader=leader,
        viewer=args.viewer,
        extra_topics=tuple(args.extra_topic),
        storage_id=args.storage_id,
        startup_timeout_s=args.startup_timeout_s,
        sample_timeout_s=args.sample_timeout_s,
        fps=args.camera_fps,
        capture_mode=args.capture_mode,
        replay_pre_roll_s=args.replay_pre_roll_s,
        replay_post_roll_s=args.replay_post_roll_s,
    )
    print(Rosbag2Collector(config).collect())


if __name__ == "__main__":
    main()
