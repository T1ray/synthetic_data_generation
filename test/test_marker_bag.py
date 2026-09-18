import gc
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from synthetic_data_generation.bag_roundtrip import roundtrip_bag
from synthetic_data_generation.ground_truth import annotation_path_for
from synthetic_data_generation.object_injector import FrameInjectionResult, ObjectInjectionStats
from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.processor import ProcessingContext
from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.smoke_test import CLOUD_TOPIC, STRING_TOPIC, make_padded_point_cloud, read_bag


def scenario(enabled):
    return parse_scenario({
        "schema_version": 2, "scenario_id": "markers", "seed": 4,
        "source": {"pointcloud_topic": CLOUD_TOPIC},
        "frames": {"start_index": 1, "end_index": 3},
        "visualization": {"enabled": enabled, "marker_topic": "/synthetic/markers",
            "point_size_m": .06, "marker_lifetime_sec": .25, "show_geometry": True,
            "show_modified_points": True, "show_text": True, "show_bounding_box": True},
        "objects": [
            {"id": "box-a", "class_name": "obstacle", "geometry": {"type": "box", "dimensions_m": [1,1,1]},
             "pose": {"coordinate_system": "lidar", "xyz_m": [10,0,0], "rpy_deg": [0,0,0]},
             "visualization": {"color_rgba": [1,0,0,.8]}},
            {"id": "box-b", "class_name": "obstacle", "geometry": {"type": "box", "dimensions_m": [1,1,1]},
             "pose": {"coordinate_system": "lidar", "xyz_m": [15,0,0], "rpy_deg": [0,0,0]},
             "visualization": {"color_rgba": [0,0,1,.8]}},
        ],
        "sensor_effects": {"range_noise": False, "dropout": False, "modify_intensity": False},
    })


def create_bag(path: Path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=CLOUD_TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name=STRING_TOPIC, type="std_msgs/msg/String", serialization_format="cdr"))
    for index in range(5):
        writer.write(CLOUD_TOPIC, serialize_message(make_padded_point_cloud([(20+index,0,0)], sequence=index)), 100+10*index)
        writer.write(STRING_TOPIC, serialize_message(String(data=str(index))), 105+10*index)
    writer.close()


def fake_inject(message, geometries):
    decoded = decode_cloud(message)
    decoded.points["x"][0,0] = 9.5
    encode_cloud(message, decoded)
    return FrameInjectionResult(1,1,0,2,1,np.array([0]),np.array([0]),np.array([[9.5,0,0.0]]),
        np.array([geometries[0].object_id],dtype=object),
        (ObjectInjectionStats(geometries[0].object_id,1,1,1), ObjectInjectionStats(geometries[1].object_id,1,0,0)))


def test_marker_messages_follow_cloud_and_disabled_mode_is_identical(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("synthetic_data_generation.processor.inject_objects", fake_inject)
    input_bag = tmp_path / "input"
    create_bag(input_bag)
    outputs = [tmp_path / "enabled", tmp_path / "disabled"]
    results = []
    records = []
    for enabled, output in zip((True, False), outputs):
        results.append(roundtrip_bag(input_bag, output, progress_every=0,
                         context=ProcessingContext.from_scenario(scenario(enabled))))
        topics, current = read_bag(output)
        records.append(current)
        if enabled:
            assert topics["/synthetic/markers"] == "visualization_msgs/msg/MarkerArray"
        else:
            assert "/synthetic/markers" not in topics

    assert results[0].input_message_count == 10
    assert results[0].output_message_count == 13
    assert results[1].output_message_count == 10
    enabled_records = records[0]
    for index, item in enumerate(enabled_records[:-1]):
        if item.topic_name == CLOUD_TOPIC and item.timestamp in {110,120,130}:
            marker = enabled_records[index+1]
            assert marker.topic_name == "/synthetic/markers"
            assert marker.timestamp == item.timestamp
            assert all(value.header.frame_id == item.message.header.frame_id for value in marker.message.markers)
            assert all(value.header.stamp == item.message.header.stamp for value in marker.message.markers)
    clouds_enabled = [bytes(r.message.data) for r in records[0] if r.topic_name == CLOUD_TOPIC]
    clouds_disabled = [bytes(r.message.data) for r in records[1] if r.topic_name == CLOUD_TOPIC]
    assert clouds_enabled == clouds_disabled
    assert annotation_path_for(outputs[0]).read_bytes() == annotation_path_for(outputs[1]).read_bytes()
    annotations = [json.loads(line) for line in annotation_path_for(outputs[0]).read_text(encoding="utf-8").splitlines()]
    assert [item["frame_index"] for item in annotations] == [1, 2, 3]
    for annotation in annotations:
        assert [item["object_id"] for item in annotation["objects"]] == ["box-a", "box-b"]
        assert [item["modified_slot_count"] for item in annotation["objects"]] == [1, 0]
        assert sum(item["modified_slot_count"] for item in annotation["objects"]) == annotation["modified_slot_count"]
    del records
    gc.collect()
