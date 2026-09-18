import copy

import numpy as np
import pytest

from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.scenario import ZeroSlotRecoveryConfig
from synthetic_data_generation.smoke_test import make_padded_point_cloud
from synthetic_data_generation.zero_slot_recovery import DirectionRecoveryError, build_rays_with_zero_recovery


def recovery_config(**changes):
    values=dict(enabled=True,ring_field="ring",timestamp_field="timestamp",timestamp_unit="microseconds",
                min_valid_samples_per_ring=4,interpolation="linear",allow_extrapolation=False,max_angular_error_deg=.1)
    values.update(changes); return ZeroSlotRecoveryConfig(**values)


def synthetic_scan():
    angles=np.array([2.8,3.0,3.1,-3.1,-3.0,-2.8])
    truth=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles))))
    cloud=make_padded_point_cloud(20*truth)
    decoded=decode_cloud(cloud)
    decoded.points["ring"][:]=7
    decoded.points["timestamp"][:]=np.arange(len(angles),dtype=float)
    decoded.points["x"][0,2:4]=0; decoded.points["y"][0,2:4]=0; decoded.points["z"][0,2:4]=0
    encode_cloud(cloud,decoded)
    return cloud,truth


def test_recovers_zero_slots_through_pi_wrap_with_low_error():
    cloud,truth=synthetic_scan(); decoded=decode_cloud(cloud)
    ring_before=decoded.points["ring"].copy(); time_before=decoded.points["timestamp"].copy()
    result=build_rays_with_zero_recovery(decoded,recovery_config())
    assert result.stats.zero_slot_count==2
    assert result.stats.recovered_direction_count==2
    recovered=result.directions[result.recovered_for_rays]
    errors=np.degrees(np.arccos(np.clip(np.sum(recovered*truth[2:4],axis=1),-1,1)))
    assert np.max(errors)<.5
    assert np.array_equal(decoded.points["ring"],ring_before)
    assert np.array_equal(decoded.points["timestamp"],time_before)


def test_no_extrapolation_and_insufficient_ring_are_reported():
    cloud,_=synthetic_scan(); decoded=decode_cloud(cloud)
    decoded.points["timestamp"][0,2]=-10
    result=build_rays_with_zero_recovery(decoded,recovery_config())
    assert result.stats.recovered_direction_count==1
    assert result.stats.rejection_reasons["timestamp_out_of_range"]==1
    result=build_rays_with_zero_recovery(decoded,recovery_config(min_valid_samples_per_ring=5))
    assert result.stats.recovered_direction_count==0
    assert result.stats.rejection_reasons["insufficient_valid_samples"]==2


def test_missing_ring_has_clear_error_and_no_zero_slots_invent_nothing():
    cloud,_=synthetic_scan(); decoded=decode_cloud(cloud)
    with pytest.raises(DirectionRecoveryError,match="missing_ring"):
        build_rays_with_zero_recovery(decoded,recovery_config(ring_field="missing_ring"))
    full=decode_cloud(make_padded_point_cloud([(10,0,0),(20,0,0)]))
    result=build_rays_with_zero_recovery(full,recovery_config())
    assert result.stats.zero_slot_count==0 and result.stats.recovered_direction_count==0
