"""Multi-object Open3D ray casting with first-hit ownership."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2

from synthetic_data_generation.cube_injector import Open3DUnavailableError
from synthetic_data_generation.geometry import BuiltGeometry
from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.ray_model import build_valid_rays


@dataclass(frozen=True)
class ObjectInjectionStats:
    object_id: str
    ray_intersection_count: int
    visible_point_count: int
    modified_slot_count: int


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
) -> FrameInjectionResult:
    """Replace returns using the nearest synthetic hit across all objects."""
    decoded = decode_cloud(message)
    rays = build_valid_rays(decoded)
    empty_int = np.empty(0, dtype=np.int64)
    empty_xyz = np.empty((0, 3), dtype=np.float64)
    empty_ids = np.empty(0, dtype=object)
    if not len(rays.ranges):
        return FrameInjectionResult(
            rays.total_points, 0, rays.invalid_points, 0, 0,
            empty_int, empty_int.copy(), empty_xyz, empty_ids,
            tuple(ObjectInjectionStats(item.object_id, 0, 0, 0) for item in geometries),
        )

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
    replace_mask = hit_mask & (hit_distances < rays.ranges - epsilon)

    hit_object_ids = np.empty(len(rays.ranges), dtype=object)
    hit_object_ids[:] = None
    for geometry_id in np.unique(geometry_ids[hit_mask]):
        object_id = geometry_id_to_object.get(int(geometry_id))
        if object_id is None:
            raise RuntimeError(f"Cannot map Open3D geometry ID {int(geometry_id)} to an object ID")
        hit_object_ids[hit_mask & (geometry_ids == geometry_id)] = object_id

    rows = rays.rows[replace_mask]
    columns = rays.columns[replace_mask]
    modified_xyz = rays.directions[replace_mask] * hit_distances[replace_mask, None]
    winning_ids = hit_object_ids[replace_mask].copy()
    if len(modified_xyz):
        decoded.points["x"][rows, columns] = modified_xyz[:, 0]
        decoded.points["y"][rows, columns] = modified_xyz[:, 1]
        decoded.points["z"][rows, columns] = modified_xyz[:, 2]
    encode_cloud(message, decoded)

    object_stats = tuple(
        ObjectInjectionStats(
            item.object_id,
            int(np.count_nonzero(hit_mask & (hit_object_ids == item.object_id))),
            int(np.count_nonzero(winning_ids == item.object_id)),
            int(np.count_nonzero(winning_ids == item.object_id)),
        )
        for item in geometries
    )
    return FrameInjectionResult(
        rays.total_points, len(rays.ranges), rays.invalid_points,
        int(np.count_nonzero(hit_mask)), int(np.count_nonzero(replace_mask)),
        rows, columns, modified_xyz, winning_ids, object_stats,
    )
