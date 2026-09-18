import copy

import pytest

from synthetic_data_generation.geometry import build_geometries
from synthetic_data_generation.object_injector import inject_objects
from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.smoke_test import make_padded_point_cloud
from test_geometry_v2 import obj, v2_document


def _require_open3d():
    return pytest.importorskip("open3d", reason="Open3D is not installed")


def test_nearest_synthetic_object_wins_and_background_occludes():
    _require_open3d()
    scenario = parse_scenario(v2_document([
        obj("near", {"type": "box", "dimensions_m": [2,2,2]}, xyz=(10,0,-1)),
        obj("far", {"type": "cylinder", "radius_m": 1, "height_m": 2, "radial_segments": 32}, xyz=(15,0,-1)),
    ], enabled=False))
    geometries = build_geometries(scenario.objects)
    cloud = make_padded_point_cloud([(20,0,0), (5,0,0)])
    original_second = copy.deepcopy(cloud)
    result = inject_objects(cloud, geometries)
    assert result.modified_slot_count == 1
    assert result.winning_object_ids.tolist() == ["near"]
    stats = {item.object_id: item for item in result.object_stats}
    assert stats["near"].modified_slot_count == 1
    assert stats["far"].modified_slot_count == 0
    assert bytes(cloud.data)[cloud.point_step:] == bytes(original_second.data)[cloud.point_step:]


def test_thin_straight_cable_can_be_hit():
    _require_open3d()
    scenario = parse_scenario(v2_document([obj("cable", {
        "type": "cable", "radius_m": .05, "radial_segments": 16,
        "control_points_m": [[0,-1,0],[0,1,0]],
    }, xyz=(10,0,0))], enabled=False))
    result = inject_objects(make_padded_point_cloud([(20,0,0)]), build_geometries(scenario.objects))
    assert result.modified_slot_count == 1
    assert result.winning_object_ids.tolist() == ["cable"]


@pytest.mark.parametrize("geometry,xyz,rpy,point,expected", [
    ({"type": "box", "dimensions_m": [2,1,2]}, (10,0,-1), (0,0,45), (20,0,0), 1),
    ({"type": "cylinder", "radius_m": 1, "height_m": 2, "radial_segments": 32}, (10,0,-1), (0,0,0), (20,0,0), 1),
    ({"type": "cylinder", "radius_m": 1, "height_m": 2, "radial_segments": 32}, (0,0,10), (0,0,0), (0,0,20), 1),
    ({"type": "cylinder", "radius_m": 1, "height_m": 2, "radial_segments": 32}, (10,0,0), (0,90,0), (20,0,0), 1),
    ({"type": "cylinder", "radius_m": .5, "height_m": 2, "radial_segments": 32}, (10,2,-1), (0,0,0), (20,0,0), 0),
])
def test_rotated_box_and_cylinder_side_cap_rotation_and_miss(geometry, xyz, rpy, point, expected):
    _require_open3d()
    scenario = parse_scenario(v2_document([obj("shape", geometry, xyz=xyz, rpy=rpy)], enabled=False))
    result = inject_objects(make_padded_point_cloud([point]), build_geometries(scenario.objects))
    assert result.modified_slot_count == expected
