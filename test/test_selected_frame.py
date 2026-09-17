import gc
from pathlib import Path
from types import SimpleNamespace

import pytest
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from synthetic_data_generation.bag_roundtrip import RoundtripError, roundtrip_bag
from synthetic_data_generation.cube_injector import CubeConfig
from synthetic_data_generation.processor import ProcessingContext
from synthetic_data_generation.smoke_test import (
    CLOUD_TOPIC,
    STRING_TOPIC,
    make_padded_point_cloud,
    read_bag,
)


def _create_three_frame_bag(path: Path) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=STRING_TOPIC,
            type="std_msgs/msg/String",
            serialization_format="cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=CLOUD_TOPIC,
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr",
        )
    )
    messages = [
        (CLOUD_TOPIC, make_padded_point_cloud([(20.0, 0.0, 0.0)], sequence=0), 10),
        (STRING_TOPIC, String(data="between"), 20),
        (CLOUD_TOPIC, make_padded_point_cloud([(20.0, 0.0, 0.0)], sequence=1), 30),
        (CLOUD_TOPIC, make_padded_point_cloud([(20.0, 0.0, 0.0)], sequence=2), 40),
    ]
    for topic, message, timestamp in messages:
        writer.write(topic, serialize_message(message), timestamp)
    writer.close()


def test_missing_selected_topic_is_rejected_before_writing():
    context = ProcessingContext(
        pointcloud_topic="/missing",
        target_frame_index=0,
        cube=CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    topics = [SimpleNamespace(name=CLOUD_TOPIC, type="sensor_msgs/msg/PointCloud2")]
    with pytest.raises(RoundtripError, match="was not found"):
        context.validate_topics(topics)


def test_missing_target_frame_is_reported():
    context = ProcessingContext(
        pointcloud_topic=CLOUD_TOPIC,
        target_frame_index=3,
        cube=CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    context.frames_seen = 2
    with pytest.raises(RoundtripError, match="observed 2 frames"):
        context.finalize()


def test_only_selected_pointcloud_frame_is_modified(tmp_path: Path):
    pytest.importorskip("open3d", reason="Open3D is not installed")
    input_bag = tmp_path / "input"
    output_bag = tmp_path / "output"
    _create_three_frame_bag(input_bag)
    _, before = read_bag(input_bag)

    context = ProcessingContext(
        pointcloud_topic=CLOUD_TOPIC,
        target_frame_index=1,
        cube=CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    result = roundtrip_bag(input_bag, output_bag, progress_every=0, context=context)
    _, after = read_bag(output_bag)

    assert result.message_count == len(before) == len(after)
    for index, (expected, actual) in enumerate(zip(before, after)):
        assert expected.topic_name == actual.topic_name
        assert expected.timestamp == actual.timestamp
        if expected.topic_name != CLOUD_TOPIC:
            assert expected.message == actual.message

    before_clouds = [record.message for record in before if record.topic_name == CLOUD_TOPIC]
    after_clouds = [record.message for record in after if record.topic_name == CLOUD_TOPIC]
    assert bytes(before_clouds[0].data) == bytes(after_clouds[0].data)
    assert bytes(before_clouds[1].data) != bytes(after_clouds[1].data)
    assert bytes(before_clouds[2].data) == bytes(after_clouds[2].data)
    del before, after
    gc.collect()
