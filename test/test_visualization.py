import numpy as np
from dataclasses import replace
from visualization_msgs.msg import Marker

from synthetic_data_generation.geometry import build_geometries
from synthetic_data_generation.object_injector import FrameInjectionResult, ObjectInjectionStats
from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.smoke_test import make_padded_point_cloud
from synthetic_data_generation.visualization import build_marker_array
from test_geometry_v2 import obj, v2_document


def test_marker_array_uses_exact_modified_points_and_stable_metadata():
    scenario = parse_scenario(v2_document([
        obj("near", {"type": "box", "dimensions_m": [1,1,1]}, color=(1,0,0,.8)),
        obj("far", {"type": "cylinder", "radius_m": .5, "height_m": 1, "radial_segments": 16}, color=(0,0,1,.9)),
    ]))
    geometries = build_geometries(scenario.objects)
    cloud = make_padded_point_cloud([(20,0,0), (20,1,0)], sequence=3)
    result = FrameInjectionResult(
        2, 2, 0, 2, 2, np.array([0,0]), np.array([0,1]),
        np.array([[9.5,0,0],[15.5,1,0]], dtype=float), np.array(["near","far"], dtype=object),
        (ObjectInjectionStats("near",1,1,1), ObjectInjectionStats("far",1,1,1)),
    )
    array = build_marker_array(cloud, geometries, result, scenario.visualization)
    assert len(array.markers) == 8
    points = {m.ns: m for m in array.markers if m.ns.endswith("/points")}
    assert [(p.x,p.y,p.z) for p in points["synthetic/near/points"].points] == [(9.5,0.0,0.0)]
    assert points["synthetic/near/points"].color.r == 1.0
    assert points["synthetic/far/points"].color.b == 1.0
    for marker in array.markers:
        assert marker.header.frame_id == cloud.header.frame_id
        assert marker.header.stamp == cloud.header.stamp
        assert marker.lifetime.sec > 0 or marker.lifetime.nanosec > 0
        assert marker.color.a > 0
        assert marker.ns.startswith("synthetic/")


def test_zero_visible_object_still_has_empty_points_marker():
    scenario = parse_scenario(v2_document([obj("box", {"type": "box", "dimensions_m": [1,1,1]})]))
    geometry = build_geometries(scenario.objects)
    cloud = make_padded_point_cloud([(5,0,0)])
    result = FrameInjectionResult(1,1,0,0,0,np.empty(0,dtype=int),np.empty(0,dtype=int),
        np.empty((0,3)),np.empty(0,dtype=object),(ObjectInjectionStats("box",0,0,0),))
    array = build_marker_array(cloud, geometry, result, scenario.visualization)
    marker = next(item for item in array.markers if item.ns == "synthetic/box/points")
    assert len(marker.points) == 0


def test_frame_mode_clears_previous_markers_and_keeps_current_until_next_frame():
    scenario = parse_scenario(v2_document([obj("box", {"type": "box", "dimensions_m": [1,1,1]})]))
    geometry = build_geometries(scenario.objects)
    cloud = make_padded_point_cloud([(5,0,0)])
    result = FrameInjectionResult(1,1,0,0,0,np.empty(0,dtype=int),np.empty(0,dtype=int),
        np.empty((0,3)),np.empty(0,dtype=object),(ObjectInjectionStats("box",0,0,0),))
    config = replace(scenario.visualization, marker_mode="frame")
    markers = build_marker_array(cloud, geometry, result, config).markers
    assert markers[0].action == Marker.DELETEALL
    assert len(markers) == 5
    assert all(marker.action == Marker.ADD for marker in markers[1:])
    assert all(marker.lifetime.sec == marker.lifetime.nanosec == 0 for marker in markers[1:])
