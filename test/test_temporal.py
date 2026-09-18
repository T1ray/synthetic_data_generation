from pathlib import Path

import numpy as np
import pytest

from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.temporal import (
    CsvTrajectoryPoseProvider, PolylineTrack, TemporalError, TemporalScene,
)


def write_trajectory(path: Path):
    path.write_text("timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n"
                    "1000000000,0,0,0,0,0,0,1\n"
                    "2000000000,2,0,0,0,0,0.7071067811865475,0.7071067811865476\n",
                    encoding="utf-8")


def v3_document(trajectory: Path, *, motion="static", active=(0,2)):
    motion_value={"type":"static"} if motion=="static" else {
        "type":"constant_track_velocity","longitudinal_velocity_mps":1.0,
        "lateral_velocity_mps":0.0,"vertical_velocity_mps":0.0,"yaw_rate_deg_s":10.0}
    return {"schema_version":3,"scenario_id":"temporal","seed":1,
      "source":{"pointcloud_topic":"/points"},"frames":{"start_index":0,"end_index":2},
      "zero_slot_recovery":{"enabled":False,"ring_field":"ring","timestamp_field":"timestamp","timestamp_unit":"nanoseconds","min_valid_samples_per_ring":2,"interpolation":"linear","allow_extrapolation":False,"max_angular_error_deg":1.0},
      "sensor_effects":{"no_return_encoding":"zero_xyz",
        "range_noise":{"enabled":False,"model":"gaussian","base_sigma_m":0,"sigma_per_meter":0,"incidence_sigma_scale":0,"min_range_m":.1,"max_resample_attempts":0},
        "dropout":{"enabled":False,"base_return_probability":1,"distance_reference_m":30,"distance_exponent":1,"incidence_exponent":1,"min_return_probability":0},
        "intensity":{"enabled":False,"model":"empirical","field_name":"intensity","range_bin_m":5,"min_samples_per_bin":1,"incidence_exponent":1,"additive_sigma":0,"fallback":"preserve_original","on_missing":"error"}},
      "track":{"reference_frame":"map","model":{"type":"polyline","points_m":[[0,0,0],[20,0,0],[20,20,0]]},"up_hint":[0,0,1]},
      "ego_motion":{"source":"trajectory_csv","path":str(trajectory),"reference_frame":"map","lidar_frame":"lidar","timestamp_unit":"nanoseconds","max_interpolation_gap_ms":1000},
      "visualization":{"enabled":False,"marker_topic":"/synthetic/markers","point_size_m":.05,"marker_lifetime_sec":.2},
      "objects":[{"id":"track-box","class_name":"obstacle","geometry":{"type":"box","dimensions_m":[1,1,1]},
        "placement":{"type":"track_relative","longitudinal_m":10,"lateral_m":1,"height_m":0,"rpy_track_deg":[0,0,0]},
        "temporal":{"active_frames":{"start_index":active[0],"end_index":active[1]},"motion":motion_value},
        "material":{"reflectivity":.5,"return_probability_scale":1},"visualization":{"color_rgba":[1,0,0,1]}}]}


def test_polyline_uses_arc_length_and_left_handed_direction_is_correct(tmp_path: Path):
    trajectory=tmp_path/"trajectory.csv"; write_trajectory(trajectory)
    scenario=parse_scenario(v3_document(trajectory),base_dir=tmp_path)
    track=PolylineTrack(scenario.track)
    first=track.frame_at(10); corner=track.frame_at(25)
    np.testing.assert_allclose(first.center,[10,0,0]); np.testing.assert_allclose(first.lateral,[0,1,0])
    np.testing.assert_allclose(corner.center,[20,5,0]); np.testing.assert_allclose(corner.tangent,[0,1,0])
    basis=np.column_stack((corner.tangent,corner.lateral,corner.up))
    np.testing.assert_allclose(basis.T@basis,np.eye(3),atol=1e-12); assert np.linalg.det(basis)>0
    with pytest.raises(TemporalError,match="outside"): track.frame_at(100)


def test_trajectory_translation_slerp_and_no_extrapolation(tmp_path: Path):
    trajectory=tmp_path/"trajectory.csv"; write_trajectory(trajectory)
    scenario=parse_scenario(v3_document(trajectory),base_dir=tmp_path)
    provider=CsvTrajectoryPoseProvider(scenario.ego_motion)
    pose=provider.pose_at(1_500_000_000)
    np.testing.assert_allclose(pose[:3,3],[1,0,0])
    direction=pose[:3,:3]@np.array([1,0,0]); np.testing.assert_allclose(direction[:2],[np.sqrt(.5),np.sqrt(.5)],atol=1e-6)
    with pytest.raises(TemporalError,match="extrapolation"): provider.pose_at(0)


def test_reference_to_lidar_transform_motion_and_active_boundaries(tmp_path: Path):
    trajectory=tmp_path/"trajectory.csv"; write_trajectory(trajectory)
    scenario=parse_scenario(v3_document(trajectory,motion="velocity",active=(1,2)),base_dir=tmp_path)
    scene=TemporalScene(scenario)
    assert not scene.states_at(0,1_000_000_000)[0].active
    start=scene.states_at(1,1_000_000_000)[0]
    later=scene.states_at(2,2_000_000_000)[0]
    assert start.active and later.active
    np.testing.assert_allclose(start.transform_reference[:3,3],[10,1,0],atol=1e-12)
    np.testing.assert_allclose(later.track_state["longitudinal_m"],11.0)
    expected=np.linalg.inv(scene.provider.pose_at(2_000_000_000))@later.transform_reference
    np.testing.assert_allclose(later.transform_lidar,expected)
    assert later.track_state["rpy_track_deg"][2]==pytest.approx(10.0)
