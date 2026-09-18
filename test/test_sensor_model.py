import copy

import numpy as np
import pytest

from synthetic_data_generation.geometry import build_geometries
from synthetic_data_generation.object_injector import inject_objects
from synthetic_data_generation.pointcloud_codec import decode_cloud
from synthetic_data_generation.scenario import (
    DropoutConfig, IntensityConfig, MaterialConfig, RangeNoiseConfig, SensorEffectsConfig,
)
from synthetic_data_generation.sensor_model import SensorModelError
from synthetic_data_generation.smoke_test import make_padded_point_cloud
from test_geometry_v2 import obj, v2_document
from synthetic_data_generation.scenario import parse_scenario


def geometry():
    scenario=parse_scenario(v2_document([obj("box",{"type":"box","dimensions_m":[2,2,2]},xyz=(10,0,-1))],enabled=False))
    return build_geometries(scenario.objects),{"box":MaterialConfig(.5,.8)}


def effects(noise=False,dropout=False,intensity=False,probability=1.0):
    return SensorEffectsConfig("zero_xyz",
        RangeNoiseConfig(noise,"gaussian",.05,0,0,.2,8),
        DropoutConfig(dropout,probability,30,0,0,0),
        IntensityConfig(intensity,"empirical","intensity",5,1,1,0,"preserve_original","error",0))


def test_noise_is_reproducible_changes_range_not_direction_and_stays_before_background():
    built,materials=geometry(); outputs=[]
    for _ in range(2):
        cloud=make_padded_point_cloud([(20,0,0),(25,0,0)])
        result=inject_objects(cloud,built,scenario_seed=99,frame_index=3,sensor_effects=effects(noise=True),materials=materials)
        outputs.append(bytes(cloud.data))
        decoded=decode_cloud(cloud); xyz=np.column_stack((decoded.points["x"].ravel(),decoded.points["y"].ravel(),decoded.points["z"].ravel()))
        assert np.all(np.linalg.norm(xyz,axis=1)<np.array([20,25]))
        assert np.allclose(xyz[:,1:],0,atol=1e-6)
        assert result.sensor_effect_stats.range_noise_applied_count==2
    assert outputs[0]==outputs[1]


def test_dropout_zeroes_xyz_and_intensity_but_preserves_ring_and_timestamp():
    built,materials=geometry(); cloud=make_padded_point_cloud([(20,0,0),(25,0,0)])
    before=decode_cloud(copy.deepcopy(cloud))
    result=inject_objects(cloud,built,scenario_seed=1,frame_index=0,
        sensor_effects=effects(dropout=True,probability=0.0),materials=materials)
    after=decode_cloud(cloud)
    assert result.sensor_effect_stats.dropout_count==2
    assert np.all(after.points["x"]==0) and np.all(after.points["intensity"]==0)
    assert np.array_equal(after.points["ring"],before.points["ring"])
    assert np.array_equal(after.points["timestamp"],before.points["timestamp"])


def test_intensity_only_changes_returned_hits_and_does_not_change_dropout_mask():
    built,materials=geometry(); masks=[]
    for enabled in (False,True):
        cloud=make_padded_point_cloud([(20,0,0),(20,.2,0),(20,5,0)])
        before=decode_cloud(copy.deepcopy(cloud)).points["intensity"].copy()
        config=effects(dropout=True,intensity=enabled,probability=.5)
        result=inject_objects(cloud,built,scenario_seed=123,frame_index=4,sensor_effects=config,materials=materials)
        masks.append(result.returned_after_dropout_mask.copy())
        after=decode_cloud(cloud).points["intensity"]
        assert after[0,2]==before[0,2]
    assert np.array_equal(masks[0],masks[1])


def test_missing_intensity_field_errors_without_changing_layout():
    built,materials=geometry(); cloud=make_padded_point_cloud([(20,0,0)])
    cloud.fields=[field for field in cloud.fields if field.name!="intensity"]
    with pytest.raises(SensorModelError,match="requires field"):
        inject_objects(cloud,built,sensor_effects=effects(intensity=True),materials=materials)
