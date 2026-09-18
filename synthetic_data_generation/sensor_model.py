"""Deterministic range-noise, dropout, intensity, and no-return encoding."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib

import numpy as np

from synthetic_data_generation.pointcloud_codec import DecodedPointCloud
from synthetic_data_generation.scenario import MaterialConfig, SensorEffectsConfig
from synthetic_data_generation.zero_slot_recovery import RecoveredRayBundle


class SensorModelError(ValueError):
    pass


NOISE_EFFECT=101
DROPOUT_EFFECT=202
INTENSITY_EFFECT=303


def stable_object_key(object_id: str) -> int:
    return int.from_bytes(hashlib.sha256(object_id.encode("utf-8")).digest()[:4],"little")


def effect_rng(seed: int, frame_index: int, object_id: str, effect_code: int) -> np.random.Generator:
    return np.random.default_rng(np.random.SeedSequence([seed,frame_index,stable_object_key(object_id),effect_code]))


@dataclass(frozen=True)
class SensorEffectStats:
    range_noise_applied_count: int
    range_noise_resample_count: int
    range_noise_clamp_count: int
    dropout_count: int
    intensity_generated_count: int
    intensity_fallback_counts: dict[str,int]
    normal_fallback_count: int


@dataclass(frozen=True)
class SensorModelResult:
    returned_mask: np.ndarray
    dropped_mask: np.ndarray
    noisy_ranges_m: np.ndarray
    incidence_cosine: np.ndarray
    returned_xyz: np.ndarray
    stats: SensorEffectStats


def _intensity_limits(dtype: np.dtype) -> tuple[float,float]:
    if np.issubdtype(dtype,np.integer):
        info=np.iinfo(dtype); return float(info.min),float(info.max)
    if np.issubdtype(dtype,np.floating):
        info=np.finfo(dtype); return float(info.min),float(info.max)
    raise SensorModelError(f"unsupported intensity dtype {dtype}")


def apply_sensor_model(decoded: DecodedPointCloud, rays: RecoveredRayBundle,
                       ideal_mask: np.ndarray, ideal_ranges: np.ndarray,
                       primitive_normals: np.ndarray, winning_object_ids: np.ndarray,
                       materials: dict[str,MaterialConfig], config: SensorEffectsConfig,
                       *, seed: int, frame_index: int, epsilon: float=1e-5) -> SensorModelResult:
    count=len(rays.rows); noisy=ideal_ranges.copy()
    normals=np.asarray(primitive_normals,dtype=np.float64)
    normal_norm=np.linalg.norm(normals,axis=1)
    normal_valid=np.isfinite(normals).all(axis=1)&(normal_norm>1e-12)
    incidence=np.ones(count,dtype=np.float64)
    incidence[normal_valid]=np.abs(np.sum(-rays.directions[normal_valid]*(normals[normal_valid]/normal_norm[normal_valid][:,None]),axis=1))
    incidence=np.clip(incidence,0.0,1.0)
    normal_fallback=int(np.count_nonzero(ideal_mask&~normal_valid))
    resamples=clamps=noise_count=0

    for object_id in materials:
        mask=ideal_mask&(winning_object_ids==object_id)
        if not np.any(mask): continue
        indices=np.flatnonzero(mask); noise=config.range_noise
        if noise.enabled:
            sigma=noise.base_sigma_m+noise.sigma_per_meter*ideal_ranges[indices]+noise.incidence_sigma_scale*(1-incidence[indices])
            rng=effect_rng(seed,frame_index,object_id,NOISE_EFFECT)
            candidate=ideal_ranges[indices]+rng.normal(0.0,sigma)
            upper=rays.background_ranges[indices]-epsilon
            acceptable=np.isfinite(candidate)&(candidate>=noise.min_range_m)&(candidate<=upper)
            for _ in range(noise.max_resample_attempts):
                if np.all(acceptable): break
                retry=~acceptable; resamples+=int(np.count_nonzero(retry))
                candidate[retry]=ideal_ranges[indices][retry]+rng.normal(0.0,sigma[retry])
                acceptable=np.isfinite(candidate)&(candidate>=noise.min_range_m)&(candidate<=upper)
            if not np.all(acceptable):
                retry=~acceptable; clamps+=int(np.count_nonzero(retry))
                candidate[retry]=np.maximum(noise.min_range_m,np.minimum(ideal_ranges[indices][retry],upper[retry]))
            noisy[indices]=candidate; noise_count+=len(indices)

    returned=ideal_mask.copy()
    for object_id,material in materials.items():
        indices=np.flatnonzero(ideal_mask&(winning_object_ids==object_id))
        if not len(indices): continue
        dropout=config.dropout
        if dropout.enabled:
            distance_factor=np.minimum(1.0,(dropout.distance_reference_m/np.maximum(noisy[indices],epsilon))**dropout.distance_exponent)
            probability=dropout.base_return_probability*distance_factor*(incidence[indices]**dropout.incidence_exponent)*material.return_probability_scale
            probability=np.clip(probability,dropout.min_return_probability,1.0)
            returned[indices]=effect_rng(seed,frame_index,object_id,DROPOUT_EFFECT).random(len(indices))<probability
    dropped=ideal_mask&~returned
    original_xyz=np.stack([np.asarray(decoded.points[name],dtype=np.float64).copy() for name in ("x","y","z")],axis=-1)
    returned_xyz=rays.directions[returned]*noisy[returned][:,None]
    if np.any(returned):
        rows=rays.rows[returned]; columns=rays.columns[returned]
        decoded.points["x"][rows,columns]=returned_xyz[:,0]
        decoded.points["y"][rows,columns]=returned_xyz[:,1]
        decoded.points["z"][rows,columns]=returned_xyz[:,2]
    if np.any(dropped):
        rows=rays.rows[dropped]; columns=rays.columns[dropped]
        decoded.points["x"][rows,columns]=0; decoded.points["y"][rows,columns]=0; decoded.points["z"][rows,columns]=0

    intensity_count=0; fallbacks: dict[str,int]={}
    intensity=config.intensity; names=decoded.points.dtype.names or ()
    if intensity.enabled and intensity.field_name not in names:
        if intensity.on_missing=="error": raise SensorModelError(f"intensity effect requires field {intensity.field_name!r}")
        fallbacks["missing_field_skipped"]=int(np.count_nonzero(returned))
    elif intensity.enabled:
        field=decoded.points[intensity.field_name]
        if field.dtype.subdtype is not None or field.dtype.fields is not None:
            raise SensorModelError(f"intensity field {intensity.field_name!r} must be scalar")
        donor_mask=rays.original_valid_mask&np.isfinite(np.asarray(field,dtype=np.float64))
        donor_values=np.asarray(field[donor_mask],dtype=np.float64)
        donor_ranges=np.linalg.norm(original_xyz[donor_mask],axis=1)
        lower,upper=_intensity_limits(field.dtype)
        for object_id,material in materials.items():
            indices=np.flatnonzero(returned&(winning_object_ids==object_id))
            if not len(indices): continue
            rng=effect_rng(seed,frame_index,object_id,INTENSITY_EFFECT)
            values=np.empty(len(indices),dtype=np.float64)
            for output_index,ray_index in enumerate(indices):
                target_bin=int(noisy[ray_index]//intensity.range_bin_m)
                donor_bins=(donor_ranges//intensity.range_bin_m).astype(np.int64)
                exact=np.flatnonzero(donor_bins==target_bin)
                if len(exact)>=intensity.min_samples_per_bin:
                    base=donor_values[rng.choice(exact)]; key="same_range_bin"
                elif len(donor_values):
                    nearest=np.flatnonzero(np.abs(donor_bins-target_bin)==np.min(np.abs(donor_bins-target_bin)))
                    base=donor_values[rng.choice(nearest)]; key="nearest_range_bin"
                elif intensity.fallback=="preserve_original":
                    base=float(field[rays.rows[ray_index],rays.columns[ray_index]]); key="preserve_original"
                else:
                    base=intensity.constant; key="constant"
                fallbacks[key]=fallbacks.get(key,0)+1
                values[output_index]=base*material.reflectivity*(incidence[ray_index]**intensity.incidence_exponent)+rng.normal(0.0,intensity.additive_sigma)
            values=np.clip(values,lower,upper)
            field[rays.rows[indices],rays.columns[indices]]=values.astype(field.dtype)
            intensity_count+=len(indices)
    if np.any(dropped) and intensity.field_name in names:
        decoded.points[intensity.field_name][rays.rows[dropped],rays.columns[dropped]]=0
    return SensorModelResult(returned,dropped,noisy,incidence,returned_xyz,
        SensorEffectStats(noise_count,resamples,clamps,int(np.count_nonzero(dropped)),intensity_count,fallbacks,normal_fallback))
