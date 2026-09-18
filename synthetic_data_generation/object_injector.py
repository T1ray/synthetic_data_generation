"""Multi-object Open3D ray casting with first-hit ownership."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from sensor_msgs.msg import PointCloud2

from synthetic_data_generation.cube_injector import Open3DUnavailableError
from synthetic_data_generation.geometry import BuiltGeometry
from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.scenario import MaterialConfig, SensorEffectsConfig, ZeroSlotRecoveryConfig
from synthetic_data_generation.sensor_model import SensorEffectStats, apply_sensor_model
from synthetic_data_generation.zero_slot_recovery import DirectionRecoveryStats, build_rays_with_zero_recovery


@dataclass(frozen=True)
class ObjectInjectionStats:
    object_id: str
    ray_intersection_count: int
    visible_point_count: int
    modified_slot_count: int
    ideal_visible_hit_count: int = 0
    returned_after_dropout_count: int = 0
    dropout_count: int = 0
    synthetic_returns_from_zero_slots: int = 0


@dataclass(frozen=True)
class FrameInjectionResult:
    total_points: int
    valid_ray_count: int
    invalid_point_count: int
    ray_intersection_count: int
    modified_slot_count: int
    modified_rows: np.ndarray
    modified_columns: np.ndarray
    modified_xyz: np.ndarray
    winning_object_ids: np.ndarray
    object_stats: tuple[ObjectInjectionStats, ...]
    original_valid_mask: Optional[np.ndarray] = None
    recovered_direction_mask: Optional[np.ndarray] = None
    synthetic_hit_mask: Optional[np.ndarray] = None
    visible_before_sensor_effects_mask: Optional[np.ndarray] = None
    returned_after_dropout_mask: Optional[np.ndarray] = None
    dropped_synthetic_mask: Optional[np.ndarray] = None
    ideal_ranges_m: Optional[np.ndarray] = None
    noisy_ranges_m: Optional[np.ndarray] = None
    incidence_cosine: Optional[np.ndarray] = None
    zero_slot_stats: Optional[DirectionRecoveryStats] = None
    sensor_effect_stats: Optional[SensorEffectStats] = None


def _open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise Open3DUnavailableError(
            "Open3D is required for synthetic object ray casting but is not installed"
        ) from exc
    return o3d


def inject_objects(
    message: PointCloud2,
    geometries: tuple[BuiltGeometry, ...],
    *,
    epsilon: float = 1e-5,
    scenario_seed: int = 0,
    frame_index: int = 0,
    zero_slot_config: Optional[ZeroSlotRecoveryConfig] = None,
    sensor_effects: Optional[SensorEffectsConfig] = None,
    materials: Optional[dict[str, MaterialConfig]] = None,
) -> FrameInjectionResult:
    """Replace returns using the nearest synthetic hit across all objects."""
    decoded = decode_cloud(message)
    rays = build_rays_with_zero_recovery(decoded, zero_slot_config or ZeroSlotRecoveryConfig())
    empty_int = np.empty(0, dtype=np.int64)
    empty_xyz = np.empty((0, 3), dtype=np.float64)
    empty_ids = np.empty(0, dtype=object)
    if not len(rays.background_ranges):
        return FrameInjectionResult(
            rays.total_points, 0, rays.stats.invalid_nonzero_count, 0, 0,
            empty_int, empty_int.copy(), empty_xyz, empty_ids,
            tuple(ObjectInjectionStats(item.object_id, 0, 0, 0) for item in geometries),
            original_valid_mask=rays.original_valid_mask,
            recovered_direction_mask=rays.recovered_direction_mask,
            zero_slot_stats=rays.stats,
            sensor_effect_stats=SensorEffectStats(0,0,0,0,0,{},0),
        )
    if not geometries:
        ray_count=len(rays.background_ranges)
        return FrameInjectionResult(rays.total_points,ray_count,rays.stats.invalid_nonzero_count,0,0,
            empty_int,empty_int.copy(),empty_xyz,empty_ids,tuple(),
            original_valid_mask=rays.original_valid_mask,recovered_direction_mask=rays.recovered_direction_mask,
            synthetic_hit_mask=np.zeros(ray_count,dtype=bool),visible_before_sensor_effects_mask=np.zeros(ray_count,dtype=bool),
            returned_after_dropout_mask=np.zeros(ray_count,dtype=bool),dropped_synthetic_mask=np.zeros(ray_count,dtype=bool),
            ideal_ranges_m=np.full(ray_count,np.inf),noisy_ranges_m=np.full(ray_count,np.inf),
            incidence_cosine=np.ones(ray_count),zero_slot_stats=rays.stats,
            sensor_effect_stats=SensorEffectStats(0,0,0,0,0,{},0))

    o3d = _open3d()
    scene = o3d.t.geometry.RaycastingScene()
    geometry_id_to_object: dict[int, str] = {}
    for item in geometries:
        mesh = o3d.t.geometry.TriangleMesh(
            o3d.core.Tensor(item.vertices_lidar, dtype=o3d.core.Dtype.Float32),
            o3d.core.Tensor(item.triangles, dtype=o3d.core.Dtype.Int32),
        )
        geometry_id = int(scene.add_triangles(mesh))
        if geometry_id in geometry_id_to_object:
            raise RuntimeError(f"Open3D returned duplicate geometry ID {geometry_id}")
        geometry_id_to_object[geometry_id] = item.object_id

    answers = scene.cast_rays(o3d.core.Tensor(rays.rays, dtype=o3d.core.Dtype.Float32))
    hit_distances = answers["t_hit"].numpy().astype(np.float64, copy=False)
    geometry_ids = answers["geometry_ids"].numpy().astype(np.int64, copy=False)
    hit_mask = np.isfinite(hit_distances) & (hit_distances > epsilon)
    replace_mask = hit_mask & (hit_distances < rays.background_ranges - epsilon)

    hit_object_ids = np.empty(len(rays.background_ranges), dtype=object)
    hit_object_ids[:] = None
    for geometry_id in np.unique(geometry_ids[hit_mask]):
        object_id = geometry_id_to_object.get(int(geometry_id))
        if object_id is None:
            raise RuntimeError(f"Cannot map Open3D geometry ID {int(geometry_id)} to an object ID")
        hit_object_ids[hit_mask & (geometry_ids == geometry_id)] = object_id

    normals = answers["primitive_normals"].numpy().astype(np.float64, copy=False)
    material_map = materials or {item.object_id: MaterialConfig() for item in geometries}
    sensor = apply_sensor_model(decoded, rays, replace_mask, hit_distances, normals,
                                hit_object_ids, material_map,
                                sensor_effects or SensorEffectsConfig(),
                                seed=scenario_seed, frame_index=frame_index, epsilon=epsilon)
    returned_mask = sensor.returned_mask
    rows = rays.rows[returned_mask]
    columns = rays.columns[returned_mask]
    modified_xyz = sensor.returned_xyz
    winning_ids = hit_object_ids[returned_mask].copy()
    encode_cloud(message, decoded)

    object_stats = tuple(
        ObjectInjectionStats(
            item.object_id,
            int(np.count_nonzero(hit_mask & (hit_object_ids == item.object_id))),
            int(np.count_nonzero(winning_ids == item.object_id)),
            int(np.count_nonzero(winning_ids == item.object_id)),
            int(np.count_nonzero(replace_mask & (hit_object_ids == item.object_id))),
            int(np.count_nonzero(returned_mask & (hit_object_ids == item.object_id))),
            int(np.count_nonzero(sensor.dropped_mask & (hit_object_ids == item.object_id))),
            int(np.count_nonzero(returned_mask & rays.recovered_for_rays & (hit_object_ids == item.object_id))),
        )
        for item in geometries
    )
    return FrameInjectionResult(
        rays.total_points, len(rays.background_ranges), rays.stats.invalid_nonzero_count,
        int(np.count_nonzero(hit_mask)), int(np.count_nonzero(returned_mask)),
        rows, columns, modified_xyz, winning_ids, object_stats,
        original_valid_mask=rays.original_valid_mask,
        recovered_direction_mask=rays.recovered_direction_mask,
        synthetic_hit_mask=hit_mask,
        visible_before_sensor_effects_mask=replace_mask,
        returned_after_dropout_mask=returned_mask,
        dropped_synthetic_mask=sensor.dropped_mask,
        ideal_ranges_m=hit_distances,
        noisy_ranges_m=sensor.noisy_ranges_m,
        incidence_cosine=sensor.incidence_cosine,
        zero_slot_stats=rays.stats,
        sensor_effect_stats=sensor.stats,
    )
