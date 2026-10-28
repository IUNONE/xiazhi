"""Backward-compatible import for the renamed ROS 2 collector module."""

from data_pipeline.data_collection.collectors.rosbag2 import Rosbag2Collector

__all__ = ["Rosbag2Collector"]
