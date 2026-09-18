import gc
import hashlib
import json
from pathlib import Path

import pytest
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from synthetic_data_generation.bag_roundtrip import RoundtripError, roundtrip_bag
from synthetic_data_generation.cube_injector import InjectionStats
from synthetic_data_generation.ground_truth import annotation_path_for
from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.processor import ProcessingContext
from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.smoke_test import CLOUD_TOPIC, STRING_TOPIC, make_padded_point_cloud, read_bag


def scenario(start=1, end=3, seed=7):
    return parse_scenario({
        "schema_version": 1, "scenario_id": "range-test", "seed": seed,
        "source": {"pointcloud_topic": CLOUD_TOPIC},
        "frames": {"start_index": start, "end_index": end},
        "objects": [{"id": "box", "class_name": "obstacle",
                     "geometry": {"type": "box", "dimensions_m": [2, 2, 2]},
                     "pose": {"coordinate_system": "lidar", "xyz_m": [10, 0, 0], "rpy_deg": [0, 0, 0]}}],
        "sensor_effects": {"range_noise": False, "dropout": False, "modify_intensity": False},
    })


def create_five_frame_bag(path: Path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=STRING_TOPIC, type="std_msgs/msg/String", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name=CLOUD_TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
    for index in range(5):
        writer.write(CLOUD_TOPIC, serialize_message(make_padded_point_cloud([(20.0 + index, 0, 0)], sequence=index)), 100 + index * 10)
        writer.write(STRING_TOPIC, serialize_message(String(data=str(index))), 105 + index * 10)
    writer.close()


def fake_inject(message, config):
    del config
    decoded = decode_cloud(message)
    decoded.points["x"][0, 0] -= 1.0
    encode_cloud(message, decoded)
    sequence = int(message.header.stamp.nanosec // 100_000_000)
    replaced = 0 if sequence == 2 else 1
    return InjectionStats(total_points=1, valid_rays=1, invalid_points=0, cube_hits=1, replaced_points=replaced)


def test_inclusive_range_other_topics_and_jsonl(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("synthetic_data_generation.processor.inject_cube", fake_inject)
    input_bag, output_bag = tmp_path / "input", tmp_path / "output"
    create_five_frame_bag(input_bag)
    _, before = read_bag(input_bag)
    roundtrip_bag(input_bag, output_bag, progress_every=0, context=ProcessingContext.from_scenario(scenario()))
    _, after = read_bag(output_bag)
    before_clouds = [r for r in before if r.topic_name == CLOUD_TOPIC]
    after_clouds = [r for r in after if r.topic_name == CLOUD_TOPIC]
    assert [bytes(a.message.data) != bytes(b.message.data) for a, b in zip(before_clouds, after_clouds)] == [False, True, True, True, False]
    assert [(r.topic_name, r.timestamp) for r in before] == [(r.topic_name, r.timestamp) for r in after]
    assert [r.message for r in before if r.topic_name == STRING_TOPIC] == [r.message for r in after if r.topic_name == STRING_TOPIC]
    records = [json.loads(line) for line in annotation_path_for(output_bag).read_text(encoding="utf-8").splitlines()]
    assert [r["frame_index"] for r in records] == [1, 2, 3]
    assert [r["bag_timestamp_ns"] for r in records] == [110, 120, 130]
    assert records[1]["objects"][0]["visible_point_count"] == 0
    assert records[1]["objects"][0]["ray_intersection_count"] == 1
    assert "NaN" not in annotation_path_for(output_bag).read_text(encoding="utf-8")
    del before, after
    gc.collect()


@pytest.mark.parametrize("start,end", [(0, 0), (4, 4), (0, 4)])
def test_range_boundaries(tmp_path: Path, monkeypatch, start, end):
    monkeypatch.setattr("synthetic_data_generation.processor.inject_cube", fake_inject)
    input_bag, output_bag = tmp_path / "input", tmp_path / "output"
    create_five_frame_bag(input_bag)
    roundtrip_bag(input_bag, output_bag, progress_every=0, context=ProcessingContext.from_scenario(scenario(start, end)))
    assert len(annotation_path_for(output_bag).read_text(encoding="utf-8").splitlines()) == end - start + 1


def test_partial_out_of_range_is_an_error(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("synthetic_data_generation.processor.inject_cube", fake_inject)
    input_bag, output_bag = tmp_path / "input", tmp_path / "output"
    create_five_frame_bag(input_bag)
    with pytest.raises(RoundtripError, match="not fully available"):
        roundtrip_bag(input_bag, output_bag, progress_every=0, context=ProcessingContext.from_scenario(scenario(3, 6)))


def test_existing_annotation_is_rejected(tmp_path: Path):
    input_bag, output_bag = tmp_path / "input", tmp_path / "output"
    create_five_frame_bag(input_bag)
    annotation_path_for(output_bag).write_text("keep", encoding="utf-8")
    with pytest.raises(FileExistsError, match="Annotation path already exists"):
        roundtrip_bag(input_bag, output_bag, progress_every=0, context=ProcessingContext.from_scenario(scenario()))
    assert annotation_path_for(output_bag).read_text(encoding="utf-8") == "keep"


def test_reproducible_output_and_annotations(tmp_path: Path, monkeypatch):
    monkeypatch.setattr("synthetic_data_generation.processor.inject_cube", fake_inject)
    input_bag = tmp_path / "input"
    create_five_frame_bag(input_bag)
    outputs = [tmp_path / "out1", tmp_path / "out2"]
    clouds = []
    hashes = []
    for output in outputs:
        roundtrip_bag(input_bag, output, progress_every=0, context=ProcessingContext.from_scenario(scenario()))
        _, records = read_bag(output)
        clouds.append([bytes(r.message.data) for r in records if r.topic_name == CLOUD_TOPIC])
        hashes.append(hashlib.sha256(annotation_path_for(output).read_bytes()).hexdigest())
        del records
        gc.collect()
    assert clouds[0] == clouds[1]
    assert hashes[0] == hashes[1]
