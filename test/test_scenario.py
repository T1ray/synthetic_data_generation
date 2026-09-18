from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from synthetic_data_generation.bag_roundtrip import RoundtripError, build_argument_parser, build_processing_context
from synthetic_data_generation.scenario import ScenarioError, load_scenario, parse_scenario


def good_scenario():
    return {
        "schema_version": 1, "scenario_id": "test", "seed": 42,
        "source": {"pointcloud_topic": "/test/points"},
        "frames": {"start_index": 1, "end_index": 3},
        "objects": [{"id": "box-1", "class_name": "obstacle",
                     "geometry": {"type": "box", "dimensions_m": [2, 2, 2]},
                     "pose": {"coordinate_system": "lidar", "xyz_m": [10, 0, 0], "rpy_deg": [0, 0, 0]}}],
        "sensor_effects": {"range_noise": False, "dropout": False, "modify_intensity": False},
    }


def test_load_valid_scenario(tmp_path: Path):
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(good_scenario()), encoding="utf-8")
    scenario = load_scenario(path)
    assert (scenario.start_index, scenario.end_index) == (1, 3)
    assert scenario.object.dimensions_m == (2.0, 2.0, 2.0)


@pytest.mark.parametrize("mutate,match", [
    (lambda d: d.update(schema_version=4), "schema_version"),
    (lambda d: d.pop("seed"), "missing required"),
    (lambda d: d["frames"].update(start_index=-1), "frames.start_index"),
    (lambda d: d["frames"].update(start_index=4, end_index=3), "start_index"),
    (lambda d: d["objects"][0]["geometry"].update(dimensions_m=[2, -1, 2]), "dimensions_m"),
    (lambda d: d["objects"][0]["geometry"].update(type="mesh"), "geometry.type"),
    (lambda d: d["objects"][0]["pose"].update(coordinate_system="map"), "coordinate_system"),
    (lambda d: d["objects"][0]["pose"].update(rpy_deg=[0, 0, 10]), "rotated boxes"),
    (lambda d: d["sensor_effects"].update(dropout=True), "sensor_effects.dropout"),
    (lambda d: d.update(extra=True), "unsupported key"),
])
def test_invalid_scenarios_have_path_specific_errors(mutate, match):
    document = good_scenario()
    mutate(document)
    with pytest.raises(ScenarioError, match=match):
        parse_scenario(document)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_values_rejected(bad):
    document = good_scenario()
    document["objects"][0]["pose"]["xyz_m"][0] = bad
    with pytest.raises(ScenarioError, match="finite"):
        parse_scenario(document)


def test_duplicate_object_ids_are_rejected():
    document = good_scenario()
    document["objects"].append(deepcopy(document["objects"][0]))
    with pytest.raises(ScenarioError, match="duplicate object id"):
        parse_scenario(document)


def test_yaml_safe_loader_rejects_python_tags(tmp_path: Path):
    path = tmp_path / "unsafe.yaml"
    path.write_text("!!python/object/apply:os.system ['echo unsafe']", encoding="utf-8")
    with pytest.raises(ScenarioError, match="scenario YAML"):
        load_scenario(path)


def test_scenario_cannot_be_mixed_with_legacy_options(tmp_path: Path):
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(good_scenario()), encoding="utf-8")
    args = build_argument_parser().parse_args([
        "--input", "input", "--output", "output", "--scenario", str(scenario_path),
        "--inject-cube", "--pointcloud-topic", "/test/points", "--target-frame-index", "1",
    ])
    with pytest.raises(RoundtripError, match="cannot be combined"):
        build_processing_context(args)
