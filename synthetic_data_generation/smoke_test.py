"""Self-contained smoke test for the identity bag roundtrip."""

from __future__ import annotations

import array
from dataclasses import dataclass
import gc
from pathlib import Path
import struct
import tempfile
from typing import Any, Dict, List, Optional, Sequence

import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from synthetic_data_generation.bag_roundtrip import roundtrip_bag


STRING_TOPIC = "/test/text"
CLOUD_TOPIC = "/test/points"


@dataclass(frozen=True)
class BagRecord:
    topic_name: str
    type_name: str
    timestamp: int
    message: Any


def make_point_cloud(sequence: int) -> PointCloud2:
    """Create a tiny PointCloud2 without NumPy conversion."""

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
        ),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(
            name="timestamp", offset=20, datatype=PointField.FLOAT64, count=1
        ),
    ]
    point_step = 28
    values = [
        (1.0 + sequence, 2.0, 3.0, 10.5, 7, 1000.125 + sequence),
        (-4.0, 5.0 + sequence, 6.0, 21.0, 42, 1000.250 + sequence),
    ]
    payload = bytearray(point_step * len(values))
    for index, value in enumerate(values):
        struct.pack_into("<ffffH2xd", payload, index * point_step, *value)

    stamp_ns = 1_100_000_000 + sequence * 200_000_000
    return PointCloud2(
        header=Header(
            stamp=Time(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id="synthetic_lidar",
        ),
        height=1,
        width=len(values),
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(values),
        data=array.array("B", payload),
        is_dense=False,
    )


def make_padded_point_cloud(
    xyz_values: Sequence[Sequence[float]],
    *,
    height: int = 1,
    width: Optional[int] = None,
    row_padding: int = 0,
    sequence: int = 0,
) -> PointCloud2:
    """Create a multi-field cloud with point and row padding for tests."""

    if width is None:
        width = len(xyz_values) // height
    if height * width != len(xyz_values):
        raise ValueError("height*width must equal the number of xyz values")
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
        ),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(
            name="return_type", offset=18, datatype=PointField.UINT8, count=1
        ),
        PointField(
            name="timestamp", offset=20, datatype=PointField.FLOAT64, count=1
        ),
    ]
    point_step = 32
    row_step = width * point_step + row_padding
    payload = bytearray([0xA5] * (height * row_step))
    for index, xyz in enumerate(xyz_values):
        row, column = divmod(index, width)
        offset = row * row_step + column * point_step
        struct.pack_into("<fff", payload, offset, *xyz)
        struct.pack_into("<f", payload, offset + 12, 10.0 + index)
        struct.pack_into("<H", payload, offset + 16, index % 128)
        struct.pack_into("<B", payload, offset + 18, index % 3)
        struct.pack_into("<d", payload, offset + 20, 2000.0 + index * 0.001)

    stamp_ns = 2_000_000_000 + sequence * 100_000_000
    return PointCloud2(
        header=Header(
            stamp=Time(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
            frame_id="synthetic_lidar",
        ),
        height=height,
        width=width,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=row_step,
        data=array.array("B", payload),
        is_dense=False,
    )


def create_test_bag(path: Path) -> None:
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
        (STRING_TOPIC, String(data="alpha"), 1_000_000_000),
        (CLOUD_TOPIC, make_point_cloud(0), 1_100_000_000),
        (STRING_TOPIC, String(data="beta"), 1_200_000_000),
        (CLOUD_TOPIC, make_point_cloud(1), 1_300_000_000),
    ]
    for topic_name, message, timestamp in messages:
        writer.write(topic_name, serialize_message(message), timestamp)
    writer.close()


def read_bag(path: Path) -> tuple[Dict[str, str], List[BagRecord]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        metadata.name: metadata.type
        for metadata in reader.get_all_topics_and_types()
    }
    message_classes = {
        name: get_message(type_name) for name, type_name in topic_types.items()
    }

    records: List[BagRecord] = []
    while reader.has_next():
        topic_name, serialized_data, timestamp = reader.read_next()
        records.append(
            BagRecord(
                topic_name=topic_name,
                type_name=topic_types[topic_name],
                timestamp=timestamp,
                message=deserialize_message(
                    serialized_data,
                    message_classes[topic_name],
                ),
            )
        )

    del reader
    gc.collect()
    return topic_types, records


def assert_pointcloud_equal(expected: PointCloud2, actual: PointCloud2) -> None:
    assert expected.header == actual.header
    assert expected.height == actual.height
    assert expected.width == actual.width
    assert expected.fields == actual.fields
    assert expected.is_bigendian == actual.is_bigendian
    assert expected.point_step == actual.point_step
    assert expected.row_step == actual.row_step
    assert expected.is_dense == actual.is_dense
    assert bytes(expected.data) == bytes(actual.data)


def run_smoke_test() -> None:
    with tempfile.TemporaryDirectory(prefix="bag_roundtrip_smoke_") as temp_dir:
        root = Path(temp_dir)
        input_bag = root / "input_bag"
        output_bag = root / "output_bag"

        create_test_bag(input_bag)
        result = roundtrip_bag(
            input_bag,
            output_bag,
            progress_every=0,
        )

        input_topics, input_records = read_bag(input_bag)
        output_topics, output_records = read_bag(output_bag)

        assert result.storage_id == "sqlite3"
        assert result.message_count == 4
        assert input_topics == output_topics
        assert len(input_records) == len(output_records)

        for expected, actual in zip(input_records, output_records):
            assert expected.topic_name == actual.topic_name
            assert expected.type_name == actual.type_name
            assert expected.timestamp == actual.timestamp
            assert type(expected.message) is type(actual.message)
            assert expected.message == actual.message
            if isinstance(expected.message, PointCloud2):
                assert_pointcloud_equal(expected.message, actual.message)

    print("Smoke test passed: identity roundtrip is semantically equivalent.")


def main() -> int:
    run_smoke_test()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
