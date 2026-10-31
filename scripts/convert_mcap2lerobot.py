from __future__ import annotations

import argparse
import json
from pathlib import Path

from data_pipeline.data_format import (
    ImageTopicConfig,
    LeRobotConversionConfig,
    Rosbag2ToLeRobotConverter,
)


def parse_image_topic(value: str) -> ImageTopicConfig:
    """解析 name=topic 格式的图像映射。

    参数
    --------
    value : str
        LeRobot feature 后缀与 ROS 2 topic.

    返回
    --------
    ImageTopicConfig
        完成解析的图像映射.
    """
    name, separator, topic = value.partition("=")
    if not separator or not name or not topic.startswith("/"):
        raise argparse.ArgumentTypeError("图像映射必须使用 name=/ros/topic 格式")
    return ImageTopicConfig(name=name, topic=topic)


def parse_depth_topic(value: str) -> ImageTopicConfig:
    item = parse_image_topic(value)
    return ImageTopicConfig(name=item.name, topic=item.topic, is_depth=True)


def main() -> None:
    """解析 topic 映射并生成官方 LeRobotDataset。

    返回
    --------
    None
        输出目录打印到标准输出.
    """
    parser = argparse.ArgumentParser(description="Convert all MCAP episodes in a session to one LeRobot dataset")
    parser.add_argument("--mcap_dir", "--mcap-dir", "--input-bag", type=Path, required=True,
                        help="session 目录；也兼容单个 episode 目录")
    parser.add_argument("--output_dir", "--output-dir", "--output-root", type=Path, required=True,
                        help="LeRobot 数据集输出目录（必须尚不存在）")
    parser.add_argument("--repo-id", default=None, help="默认 local/<输出目录名>")
    parser.add_argument("--task", default=None, help="默认读取 session_manifest.json 的 task")
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--action-topic", default=None, help="遥操 JointState 命令 topic；默认从 session manifest 推断")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--state-field", default="position")
    parser.add_argument("--ee-pose-topic", default="/rm_driver/udp_arm_position")
    parser.add_argument("--force-topic", default="/rm_driver/udp_six_zero_force")
    parser.add_argument("--image-topic", action="append", type=parse_image_topic, default=[])
    parser.add_argument("--depth-topic", action="append", type=parse_depth_topic, default=[])
    parser.add_argument("--storage-id", choices=["sqlite3", "mcap"], default=None)
    parser.add_argument("--tolerance-s", type=float, default=0.05)
    parser.add_argument("--no-videos", action="store_true")
    args = parser.parse_args()

    manifest_path = args.mcap_dir / "session_manifest.json"
    if not manifest_path.is_file():
        manifest_path = args.mcap_dir.parent / "session_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    task = args.task or manifest.get("task")
    if not task:
        parser.error("无法从 session_manifest.json 读取 task，请指定 --task")
    image_topics = [*args.image_topic, *args.depth_topic]
    if not image_topics:
        for camera in manifest.get("cameras", []):
            name = camera["name"]
            image_topics.append(ImageTopicConfig(name, f"/{name}/color/image_raw"))
            if camera.get("enable_depth", True):
                image_topics.append(ImageTopicConfig(f"{name}_depth", f"/{name}/depth/image_raw", is_depth=True))
    if not any(not item.is_depth for item in image_topics):
        parser.error("无法推断 RGB topic，请指定 --image-topic name=/ros/topic")

    config = LeRobotConversionConfig(
        input_bag=args.mcap_dir,
        output_root=args.output_dir,
        repo_id=args.repo_id or f"local/{args.output_dir.resolve().name}",
        task=task,
        fps=args.fps,
        state_topic=args.state_topic,
        action_topic=args.action_topic,
        state_field=args.state_field,
        ee_pose_topic=args.ee_pose_topic,
        force_topic=args.force_topic,
        image_topics=tuple(image_topics),
        storage_id=args.storage_id or "mcap",
        tolerance_s=args.tolerance_s,
        use_videos=not args.no_videos,
    )
    output_path = Rosbag2ToLeRobotConverter(config).convert()
    print(output_path)


if __name__ == "__main__":
    main()
