from __future__ import annotations

import io
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import numpy as np
from PIL import Image


@dataclass(frozen=True, slots=True)
class MessageRecord:
    """保存一条已反序列化的 rosbag2 消息。

    参数
    --------
    topic : str
        ROS 2 topic 名称.
    type_name : str
        ROS 2 消息类型名称.
    timestamp_ns : int
        rosbag2 记录时间戳.
    message : Any
        已反序列化的 ROS 2 消息对象.
    """

    topic: str
    type_name: str
    timestamp_ns: int
    message: Any


class Rosbag2Reader:
    """按 topic 读取并反序列化单个 rosbag2 episode。

    子函数
    --------
    - iter_records: 顺序读取指定 topic 的消息.
    """

    def __init__(self, bag_path: str | Path, storage_id: str | None = None) -> None:
        """初始化 rosbag2 读取器。

        参数
        --------
        bag_path : str | Path
            包含 metadata.yaml 的 rosbag2 目录.
        storage_id : str | None
            可选的 sqlite3 或 mcap 标识. 未提供时从 metadata.yaml 解析.

        返回
        --------
        None
            初始化实例状态.
        """
        self.bag_path = Path(bag_path)
        self.storage_id = storage_id or self._detect_storage_id()

    def _detect_storage_id(self) -> str:
        """从 rosbag2 metadata.yaml 解析 storage identifier。

        返回
        --------
        str
            rosbag2 storage plugin 标识.
        """
        metadata_path = self.bag_path / "metadata.yaml"
        if not metadata_path.is_file():
            raise FileNotFoundError(f"缺少 rosbag2 metadata.yaml: {metadata_path}")
        content = metadata_path.read_text(encoding="utf-8")
        match = re.search(r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)", content, re.MULTILINE)
        if match is None:
            raise ValueError(f"无法解析 storage_identifier: {metadata_path}")
        return match.group(1)

    def iter_records(self, topics: set[str]) -> Iterator[MessageRecord]:
        """顺序读取指定 topic 并反序列化 CDR 消息。

        参数
        --------
        topics : set[str]
            需要读取的 topic 集合.

        返回
        --------
        Iterator[MessageRecord]
            按 rosbag2 记录时间排序的消息迭代器.
        """
        if not self.bag_path.is_dir():
            raise NotADirectoryError(f"rosbag2 输入必须是目录: {self.bag_path}")
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise ImportError(
                "读取 rosbag2 需要已 source 的 ROS 2 Humble Python 环境"
            ) from exc

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id=self.storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        type_by_topic = {item.name: item.type for item in reader.get_all_topics_and_types()}
        missing = topics - set(type_by_topic)
        if missing:
            raise ValueError(f"rosbag2 缺少必需 topic: {sorted(missing)}")

        message_type_cache: dict[str, type[Any]] = {}
        while reader.has_next():
            topic, serialized_data, timestamp_ns = reader.read_next()
            if topic not in topics:
                continue
            type_name = type_by_topic[topic]
            if type_name not in message_type_cache:
                message_type_cache[type_name] = get_message(type_name)
            message_type = message_type_cache[type_name]
            message = deserialize_message(serialized_data, message_type)
            yield MessageRecord(
                topic=topic,
                type_name=type_name,
                timestamp_ns=timestamp_ns,
                message=message,
            )


def extract_numeric_vector(message: Any, field_path: str) -> np.ndarray:
    """从 ROS 2 消息字段提取一维浮点向量。

    参数
    --------
    message : Any
        已反序列化的 ROS 2 消息.
    field_path : str
        使用点号分隔的字段路径.

    返回
    --------
    np.ndarray
        float32 一维向量.
    """
    value = message
    for field_name in field_path.split("."):
        if not hasattr(value, field_name):
            raise AttributeError(f"消息缺少字段: {field_path}")
        value = getattr(value, field_name)
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size == 0:
        raise ValueError(f"消息字段为空: {field_path}")
    return array


def decode_rgb_image(record: MessageRecord) -> np.ndarray:
    """将 ROS 2 图像消息解码为 RGB uint8 数组。

    参数
    --------
    record : MessageRecord
        sensor_msgs/Image 或 sensor_msgs/CompressedImage 消息.

    返回
    --------
    np.ndarray
        HWC 排列的 RGB uint8 图像.
    """
    message = record.message
    if record.type_name == "sensor_msgs/msg/CompressedImage":
        with Image.open(io.BytesIO(bytes(message.data))) as image:
            return np.asarray(image.convert("RGB"), dtype=np.uint8)
    if record.type_name != "sensor_msgs/msg/Image":
        raise TypeError(f"不支持的图像消息类型: {record.type_name}")

    encoding = message.encoding.lower()
    channel_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
    }
    if encoding not in channel_by_encoding:
        raise ValueError(f"不支持的图像编码: {message.encoding}")
    channels = channel_by_encoding[encoding]
    row_width = int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(int(message.height), row_width)
    image = raw[:, : int(message.width) * channels].reshape(
        int(message.height),
        int(message.width),
        channels,
    )
    if encoding == "mono8":
        return np.repeat(image, 3, axis=2)
    if encoding in {"bgr8", "bgra8"}:
        image = image[:, :, [2, 1, 0, *([3] if channels == 4 else [])]]
    if channels == 4:
        image = image[:, :, :3]
    return np.ascontiguousarray(image, dtype=np.uint8)


def decode_depth_image(record: MessageRecord) -> np.ndarray:
    """Decode a ROS ``sensor_msgs/Image`` depth frame as uint16 millimetres."""
    if record.type_name != "sensor_msgs/msg/Image":
        raise TypeError(f"深度图仅支持 sensor_msgs/msg/Image: {record.type_name}")
    message = record.message
    encoding = message.encoding.lower()
    if encoding not in {"16uc1", "mono16"}:
        raise ValueError(f"不支持的深度图编码: {message.encoding}")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if step < width * 2:
        raise ValueError(f"深度图 step={step} 小于有效行宽 {width * 2}")
    byte_rows = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
    packed = np.ascontiguousarray(byte_rows[:, : width * 2])
    dtype = np.dtype(">u2" if bool(message.is_bigendian) else "<u2")
    depth = packed.view(dtype).reshape(height, width).astype(np.uint16, copy=False)
    return np.ascontiguousarray(depth[..., None])
