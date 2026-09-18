"""Recover directions of preserved no-return slots from ring and point time."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

import numpy as np

from synthetic_data_generation.pointcloud_codec import DecodedPointCloud
from synthetic_data_generation.scenario import ZeroSlotRecoveryConfig


class DirectionRecoveryError(ValueError):
    pass


@dataclass(frozen=True)
class DirectionRecoveryStats:
    zero_slot_count: int
    recoverable_zero_slot_count: int
    recovered_direction_count: int
    invalid_nonzero_count: int
    rejection_reasons: dict[str, int]
    median_angular_error_deg: float | None
    p95_angular_error_deg: float | None
    max_angular_error_deg: float | None


@dataclass(frozen=True)
class RecoveredRayBundle:
    rows: np.ndarray
    columns: np.ndarray
    directions: np.ndarray
    background_ranges: np.ndarray
    rays: np.ndarray
    original_valid_for_rays: np.ndarray
    recovered_for_rays: np.ndarray
    original_valid_mask: np.ndarray
    zero_slot_mask: np.ndarray
    invalid_nonzero_mask: np.ndarray
    recovered_direction_mask: np.ndarray
    total_points: int
    stats: DirectionRecoveryStats


def _angular_error(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    dots=np.sum(first*second,axis=1); return np.degrees(np.arccos(np.clip(dots,-1.0,1.0)))


def build_rays_with_zero_recovery(decoded: DecodedPointCloud, config: ZeroSlotRecoveryConfig,
                                  *, epsilon: float = 1e-4) -> RecoveredRayBundle:
    xyz=np.stack([np.asarray(decoded.points[name],dtype=np.float64) for name in ("x","y","z")],axis=-1)
    finite=np.isfinite(xyz).all(axis=-1); ranges=np.linalg.norm(xyz,axis=-1)
    valid=finite&(ranges>epsilon); zero=finite&(ranges<=epsilon); invalid=~finite
    recovered=np.zeros(valid.shape,dtype=bool); directions=np.zeros_like(xyz,dtype=np.float64)
    directions[valid]=xyz[valid]/ranges[valid][:,None]
    reasons: Counter[str]=Counter(); quality_errors=[]; recoverable=0

    if config.enabled and np.any(zero):
        names=decoded.points.dtype.names or ()
        if config.ring_field not in names:
            raise DirectionRecoveryError(f"zero-slot recovery requires field {config.ring_field!r}")
        if config.timestamp_field not in names:
            raise DirectionRecoveryError(f"zero-slot recovery requires field {config.timestamp_field!r}")
        rings=np.asarray(decoded.points[config.ring_field])
        try:
            timestamps=np.asarray(decoded.points[config.timestamp_field],dtype=np.float64)
        except (TypeError,ValueError) as exc:
            raise DirectionRecoveryError(f"timestamp field {config.timestamp_field!r} must be numeric") from exc
        if rings.shape!=zero.shape or timestamps.shape!=zero.shape or not np.issubdtype(rings.dtype,np.number):
            raise DirectionRecoveryError("ring and timestamp fields must be scalar numeric fields")
        finite_ring=np.isfinite(rings.astype(np.float64,copy=False)); finite_time=np.isfinite(timestamps)
        zero_finite_time=zero&finite_time&finite_ring
        reasons["non_finite_timestamp"]+=int(np.count_nonzero(zero&~finite_time))
        reasons["invalid_ring"]+=int(np.count_nonzero(zero&~finite_ring))
        for ring in np.unique(rings[zero_finite_time]):
            valid_ring=valid&(rings==ring)&np.isfinite(timestamps)
            target=zero_finite_time&(rings==ring)
            target_count=int(np.count_nonzero(target)); recoverable+=target_count
            if np.count_nonzero(valid_ring)<config.min_valid_samples_per_ring:
                reasons["insufficient_valid_samples"]+=target_count; continue
            valid_xyz=xyz[valid_ring]; valid_dirs=directions[valid_ring]
            elevation=np.arctan2(valid_xyz[:,2],np.linalg.norm(valid_xyz[:,:2],axis=1))
            elevation_ring=float(np.median(elevation))
            order=np.argsort(timestamps[valid_ring],kind="stable")
            times=timestamps[valid_ring][order]
            azimuth=np.unwrap(np.arctan2(valid_xyz[:,1],valid_xyz[:,0])[order])
            unique_times,inverse=np.unique(times,return_inverse=True)
            if len(unique_times)<2:
                reasons["insufficient_unique_timestamps"]+=target_count; continue
            sums=np.zeros(len(unique_times)); counts=np.zeros(len(unique_times))
            np.add.at(sums,inverse,azimuth); np.add.at(counts,inverse,1); unique_azimuth=sums/counts
            predicted_azimuth=np.interp(timestamps[valid_ring],unique_times,unique_azimuth)
            predicted=np.column_stack((np.cos(elevation_ring)*np.cos(predicted_azimuth),
                                       np.cos(elevation_ring)*np.sin(predicted_azimuth),
                                       np.full(len(predicted_azimuth),np.sin(elevation_ring))))
            errors=_angular_error(valid_dirs,predicted); quality_errors.extend(errors.tolist())
            if float(np.percentile(errors,95))>config.max_angular_error_deg:
                reasons["quality_threshold_failed"]+=target_count; continue
            target_times=timestamps[target]
            in_range=(target_times>=unique_times[0])&(target_times<=unique_times[-1])
            if not config.allow_extrapolation:
                reasons["timestamp_out_of_range"]+=int(np.count_nonzero(~in_range))
            usable=np.ones(len(target_times),dtype=bool) if config.allow_extrapolation else in_range
            target_indices=np.argwhere(target)
            if np.any(usable):
                target_azimuth=np.interp(target_times[usable],unique_times,unique_azimuth)
                target_dirs=np.column_stack((np.cos(elevation_ring)*np.cos(target_azimuth),
                                             np.cos(elevation_ring)*np.sin(target_azimuth),
                                             np.full(len(target_azimuth),np.sin(elevation_ring))))
                selected=target_indices[usable]
                directions[selected[:,0],selected[:,1]]=target_dirs
                recovered[selected[:,0],selected[:,1]]=True
    elif np.any(zero):
        reasons["recovery_disabled"]+=int(np.count_nonzero(zero))

    ray_mask=valid|recovered; rows,columns=np.nonzero(ray_mask)
    selected_directions=directions[rows,columns]
    origins=np.zeros_like(selected_directions)
    background=np.where(valid[rows,columns],ranges[rows,columns],np.inf)
    errors_array=np.asarray(quality_errors,dtype=np.float64)
    stats=DirectionRecoveryStats(int(np.count_nonzero(zero)),recoverable,int(np.count_nonzero(recovered)),
        int(np.count_nonzero(invalid)),dict(reasons),
        float(np.median(errors_array)) if len(errors_array) else None,
        float(np.percentile(errors_array,95)) if len(errors_array) else None,
        float(np.max(errors_array)) if len(errors_array) else None)
    return RecoveredRayBundle(rows,columns,selected_directions,background,
        np.concatenate((origins,selected_directions),axis=1).astype(np.float32),
        valid[rows,columns],recovered[rows,columns],valid,zero,invalid,recovered,
        decoded.height*decoded.width,stats)
