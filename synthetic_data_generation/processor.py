"""Selective coordination of cube injection within the bag roundtrip."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional

from sensor_msgs.msg import PointCloud2

from synthetic_data_generation.bag_roundtrip import RoundtripError
from synthetic_data_generation.cube_injector import (
    CubeConfig,
    InjectionStats,
    inject_cube,
)


@dataclass
class ProcessingContext:
    pointcloud_topic: str
    target_frame_index: int
    cube: CubeConfig
    frames_seen: int = 0
    injection_performed: bool = False
    injection_stats: Optional[InjectionStats] = None

    def validate_topics(self, topic_metadata: Iterable[Any]) -> None:
        topics = {topic.name: topic.type for topic in topic_metadata}
        if self.pointcloud_topic not in topics:
            raise RoundtripError(
                f"Selected PointCloud2 topic was not found: {self.pointcloud_topic}"
            )
        actual_type = topics[self.pointcloud_topic]
        if actual_type != "sensor_msgs/msg/PointCloud2":
            raise RoundtripError(
                f"Selected topic {self.pointcloud_topic!r} has type "
                f"{actual_type!r}, expected 'sensor_msgs/msg/PointCloud2'."
            )

        print(f"PointCloud2 topic: {self.pointcloud_topic}")
        print(f"Target frame index: {self.target_frame_index}")
        print(f"Cube center (LiDAR frame): {self.cube.center}")
        print(f"Cube size: {self.cube.size}")

    def process_message(self, topic_name: str, message: Any, timestamp: int) -> Any:
        del timestamp
        if topic_name != self.pointcloud_topic:
            return message
        if not isinstance(message, PointCloud2):
            raise RoundtripError(
                f"Selected topic {topic_name!r} did not deserialize as PointCloud2."
            )

        current_index = self.frames_seen
        self.frames_seen += 1
        if current_index != self.target_frame_index:
            return message

        self.injection_stats = inject_cube(message, self.cube)
        self.injection_performed = True
        stats = self.injection_stats
        print(f"Cube injection frame: {current_index}")
        print(f"  total points: {stats.total_points}")
        print(f"  valid rays: {stats.valid_rays}")
        print(f"  invalid points skipped: {stats.invalid_points}")
        print(f"  cube hits: {stats.cube_hits}")
        print(f"  replaced points after occlusion: {stats.replaced_points}")
        return message

    def finalize(self) -> None:
        if not self.injection_performed:
            raise RoundtripError(
                f"Target frame index {self.target_frame_index} was not found on "
                f"{self.pointcloud_topic}; observed {self.frames_seen} frames."
            )

