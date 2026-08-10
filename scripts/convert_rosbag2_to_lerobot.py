from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser(description="Convert one rosbag2 episode to LeRobot")
    parser.add_argument("--input-bag", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--state-field", default="position")
    parser.add_argument("--ee-pose-topic", default="/rm_driver/udp_arm_position")
    parser.add_argument("--force-topic", default="/rm_driver/udp_six_zero_force")
    parser.add_argument("--image-topic", action="append", type=parse_image_topic, required=True)
    parser.add_argument("--depth-topic", action="append", type=parse_depth_topic, default=[])
    parser.add_argument("--storage-id", choices=["sqlite3", "mcap"], default=None)
    parser.add_argument("--tolerance-s", type=float, default=0.05)
    parser.add_argument("--no-videos", action="store_true")
    args = parser.parse_args()

    config = LeRobotConversionConfig(
        input_bag=args.input_bag,
        output_root=args.output_root,
        repo_id=args.repo_id,
        task=args.task,
        fps=args.fps,
        state_topic=args.state_topic,
        state_field=args.state_field,
        ee_pose_topic=args.ee_pose_topic,
        force_topic=args.force_topic,
        image_topics=tuple([*args.image_topic, *args.depth_topic]),
        storage_id=args.storage_id,
        tolerance_s=args.tolerance_s,
        use_videos=not args.no_videos,
    )
    output_path = Rosbag2ToLeRobotConverter(config).convert()
    print(output_path)


if __name__ == "__main__":
    main()
