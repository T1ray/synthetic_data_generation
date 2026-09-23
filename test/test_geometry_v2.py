from pathlib import Path

import numpy as np
import pytest

from synthetic_data_generation.geometry import build_geometries, pose_transform, transform_points
from synthetic_data_generation.scenario import ScenarioError, parse_scenario


def v2_document(objects, *, enabled=True):
    return {
        "schema_version": 2, "scenario_id": "v2", "seed": 9,
        "source": {"pointcloud_topic": "/points"},
        "frames": {"start_index": 0, "end_index": 1},
        "visualization": {"enabled": enabled, "marker_topic": "/synthetic/markers",
            "point_size_m": 0.06, "marker_lifetime_sec": 0.25,
            "show_geometry": True, "show_modified_points": True,
            "show_text": True, "show_bounding_box": True},
        "objects": objects,
        "sensor_effects": {"range_noise": False, "dropout": False, "modify_intensity": False},
    }


def obj(object_id, geometry, xyz=(10, 0, 0), rpy=(0, 0, 0), color=(1, .2, .1, .8)):
    return {"id": object_id, "class_name": "obstacle", "geometry": geometry,
            "pose": {"coordinate_system": "lidar", "xyz_m": list(xyz), "rpy_deg": list(rpy)},
            "visualization": {"color_rgba": list(color)}}


def test_zyx_pose_rotation_and_translation():
    scenario = parse_scenario(v2_document([obj("box", {"type": "box", "dimensions_m": [2, 2, 2]}, xyz=(1,2,3), rpy=(0,0,90))]))
    transform = pose_transform(scenario.object.pose)
    point = transform_points(np.array([[1.0, 0.0, 0.0]]), transform)[0]
    np.testing.assert_allclose(point, [1.0, 3.0, 3.0], atol=1e-12)


def test_box_cylinder_and_broken_cable_build_finite_meshes():
    objects = [
        obj("box", {"type": "box", "dimensions_m": [2, 4, 6]}, rpy=(0,0,30)),
        obj("cylinder", {"type": "cylinder", "radius_m": .5, "height_m": 2, "radial_segments": 16}),
        obj("cable", {"type": "cable", "radius_m": .05, "radial_segments": 8,
                      "control_points_m": [[0,0,0], [1,0,1], [2,1,1]]}),
    ]
    built = build_geometries(parse_scenario(v2_document(objects)).objects)
    assert [item.geometry_type for item in built] == ["box", "cylinder", "cable"]
    for item in built:
        assert len(item.vertices_lidar) > 0 and len(item.triangles) > 0
        assert np.isfinite(item.vertices_lidar).all()
        assert item.triangles.min() >= 0 and item.triangles.max() < len(item.vertices_lidar)


@pytest.mark.parametrize("geometry,match", [
    ({"type": "cylinder", "radius_m": 0, "height_m": 1, "radial_segments": 16}, "radius_m"),
    ({"type": "cylinder", "radius_m": 1, "height_m": 1, "radial_segments": 4}, "radial_segments"),
    ({"type": "cable", "radius_m": .1, "radial_segments": 8, "control_points_m": [[0,0,0]]}, "at least two"),
    ({"type": "cable", "radius_m": .1, "radial_segments": 8, "control_points_m": [[0,0,0],[0,0,0]]}, "distinct"),
])
def test_geometry_validation(geometry, match):
    with pytest.raises(ScenarioError, match=match):
        parse_scenario(v2_document([obj("bad", geometry)]))


def test_duplicate_ids_and_bad_visualization_are_rejected():
    box = {"type": "box", "dimensions_m": [1,1,1]}
    with pytest.raises(ScenarioError, match="duplicate"):
        parse_scenario(v2_document([obj("same", box), obj("same", box)]))
    document = v2_document([obj("box", box)])
    document["visualization"]["marker_lifetime_sec"] = 0
    with pytest.raises(ScenarioError, match="marker_lifetime_sec"):
        parse_scenario(document)
    document["visualization"]["marker_lifetime_sec"] = 0.25
    document["visualization"]["marker_mode"] = "unknown"
    with pytest.raises(ScenarioError, match="marker_mode"):
        parse_scenario(document)


def test_human_mesh_path_and_extension_validation(tmp_path: Path):
    geometry = {"type": "human_mesh", "path": "missing.obj", "units": "meters",
                "scale": [1,1,1], "origin": "base_center"}
    with pytest.raises(ScenarioError, match="does not exist"):
        parse_scenario(v2_document([obj("human", geometry)]), base_dir=tmp_path)
