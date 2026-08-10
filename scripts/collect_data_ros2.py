from __future__ import annotations

import argparse
import uuid
from pathlib import Path

from data_pipeline.data_collection import (
    CameraConfig,
    CollectionConfig,
    RealManConfig,
    Rosbag2Collector,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record an ECO63 multi-episode session with ROS 2 rosbag2"
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--session-id", default=None)
    parser.add_argument("--camera-serial", action="append", required=True)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--drag-mode", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--disable-singular-wall", action="store_true")
    parser.add_argument("--home-return-speed", type=int, default=10)
    parser.add_argument("--home-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--extra-topic", action="append", default=[])
    parser.add_argument("--storage-id", choices=["sqlite3", "mcap"], default="mcap")
    parser.add_argument("--startup-timeout-s", type=float, default=45.0)
    parser.add_argument("--sample-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--allow-sdk-driver-coexistence",
        action="store_true",
        help=(
            "确认本机已验证 RealMan ROS driver 与 Python SDK 可安全并发连接；"
            "未提供时拒绝启动"
        ),
    )
    args = parser.parse_args()

    if len(args.camera_serial) != 3:
        parser.error("--camera-serial 必须提供三次")
    if not args.allow_sdk_driver_coexistence:
        parser.error(
            "必须先完成并发连接真机验证，再提供 --allow-sdk-driver-coexistence"
        )

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
        drag_mode=args.drag_mode,
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
        extra_topics=tuple(args.extra_topic),
        storage_id=args.storage_id,
        startup_timeout_s=args.startup_timeout_s,
        sample_timeout_s=args.sample_timeout_s,
        fps=args.camera_fps,
        allow_sdk_driver_coexistence=args.allow_sdk_driver_coexistence,
    )
    print(Rosbag2Collector(config).collect())


if __name__ == "__main__":
    main()
