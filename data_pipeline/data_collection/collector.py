"""Backward-compatible import for the renamed ROS 2 collector module."""

from data_pipeline.data_collection.ros2_collector import Rosbag2Collector

__all__ = ["Rosbag2Collector"]
