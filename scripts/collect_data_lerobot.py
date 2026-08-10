from __future__ import annotations

import argparse
import uuid
from pathlib import Path

from data_pipeline.data_collection import (
    CameraConfig,
    CollectionConfig,
    LeRobotCollector,
    RealManConfig,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record an ECO63 multi-episode session as one LeRobotDataset"
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--session-id", default=None)
    parser.add_argument(
        "--camera-serial",
        action="append",
        required=True,
        help="至少提供一次，可重复提供；按顺序映射为 camera_01、camera_02 等",
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--drag-mode", type=int, choices=[0, 1, 2, 3], default=3)
    parser.add_argument("--disable-singular-wall", action="store_true")
    parser.add_argument("--home-return-speed", type=int, default=10)
    parser.add_argument("--home-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--max-camera-age-ms", type=int, default=500)
    parser.add_argument("--no-videos", action="store_true")
    args = parser.parse_args()

    cameras = tuple(
        CameraConfig(
            name=f"camera_{index:02d}",
            serial_number=serial,
            fps=args.fps,
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
        fps=args.fps,
    )
    collector = LeRobotCollector(
        config,
        repo_id=args.repo_id,
        use_videos=not args.no_videos,
        max_camera_age_ms=args.max_camera_age_ms,
    )
    print(collector.collect())


if __name__ == "__main__":
    main()
