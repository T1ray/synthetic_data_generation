"""Injection of one axis-aligned Open3D cube with first-hit occlusion."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

import numpy as np
from sensor_msgs.msg import PointCloud2

from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.ray_model import build_valid_rays


class Open3DUnavailableError(ImportError):
    """Raised when cube injection is requested without Open3D."""


@dataclass(frozen=True)
class CubeConfig:
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]
    epsilon: float = 1e-5

    def __init__(
        self,
        center: Sequence[float],
        size: Sequence[float],
        epsilon: float = 1e-5,
    ) -> None:
        center_tuple = tuple(float(value) for value in center)
        size_tuple = tuple(float(value) for value in size)
        if len(center_tuple) != 3 or len(size_tuple) != 3:
            raise ValueError("Cube center and size must each contain three values.")
        if not all(math.isfinite(value) for value in center_tuple + size_tuple):
            raise ValueError("Cube center and size must contain only finite values.")
        if not all(value > 0.0 for value in size_tuple):
            raise ValueError("Every cube size must be positive.")
        if not math.isfinite(epsilon) or epsilon <= 0.0:
            raise ValueError("Cube occlusion epsilon must be finite and positive.")
        object.__setattr__(self, "center", center_tuple)
        object.__setattr__(self, "size", size_tuple)
        object.__setattr__(self, "epsilon", float(epsilon))


@dataclass(frozen=True)
class InjectionStats:
    total_points: int
    valid_rays: int
    invalid_points: int
    cube_hits: int
    replaced_points: int


def _open3d():
    try:
        import open3d as o3d
    except ImportError as exc:
        raise Open3DUnavailableError(
            "Open3D is required for --inject-cube but is not installed. "
            "Install the project Open3D dependency in the ROS 2 environment."
        ) from exc
    return o3d


def inject_cube(
    message: PointCloud2,
    config: CubeConfig,
) -> InjectionStats:
    """Replace only returns occluded by the nearer surface of one cube."""

    decoded = decode_cloud(message)
    rays = build_valid_rays(decoded)
    if not len(rays.ranges):
        return InjectionStats(
            total_points=rays.total_points,
            valid_rays=0,
            invalid_points=rays.invalid_points,
            cube_hits=0,
            replaced_points=0,
        )

    o3d = _open3d()
    minimum_corner = np.asarray(config.center) - np.asarray(config.size) / 2.0
    mesh = o3d.geometry.TriangleMesh.create_box(
        width=config.size[0],
        height=config.size[1],
        depth=config.size[2],
    )
    mesh.translate(minimum_corner)
    tensor_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor_mesh)

    answers = scene.cast_rays(
        o3d.core.Tensor(rays.rays, dtype=o3d.core.Dtype.Float32)
    )
    hit_distances = answers["t_hit"].numpy().astype(np.float64, copy=False)
    hit_mask = np.isfinite(hit_distances) & (hit_distances > config.epsilon)
    replace_mask = hit_mask & (
        hit_distances < rays.ranges - config.epsilon
    )

    if np.any(replace_mask):
        rows = rays.rows[replace_mask]
        columns = rays.columns[replace_mask]
        new_xyz = (
            rays.directions[replace_mask]
            * hit_distances[replace_mask, None]
        )
        decoded.points["x"][rows, columns] = new_xyz[:, 0]
        decoded.points["y"][rows, columns] = new_xyz[:, 1]
        decoded.points["z"][rows, columns] = new_xyz[:, 2]

    encode_cloud(message, decoded)
    return InjectionStats(
        total_points=rays.total_points,
        valid_rays=len(rays.ranges),
        invalid_points=rays.invalid_points,
        cube_hits=int(np.count_nonzero(hit_mask)),
        replaced_points=int(np.count_nonzero(replace_mask)),
    )

