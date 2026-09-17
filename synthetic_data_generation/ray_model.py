"""Construction of valid normalized LiDAR rays from decoded PointCloud2 data."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from synthetic_data_generation.pointcloud_codec import DecodedPointCloud


@dataclass(frozen=True)
class RayBundle:
    rows: np.ndarray
    columns: np.ndarray
    directions: np.ndarray
    ranges: np.ndarray
    rays: np.ndarray
    total_points: int
    invalid_points: int


def build_valid_rays(
    decoded: DecodedPointCloud,
    *,
    minimum_range: float = 1e-4,
) -> RayBundle:
    """Build rays only for finite points farther than ``minimum_range``."""

    x = np.asarray(decoded.points["x"], dtype=np.float64)
    y = np.asarray(decoded.points["y"], dtype=np.float64)
    z = np.asarray(decoded.points["z"], dtype=np.float64)
    xyz = np.stack((x, y, z), axis=-1)
    finite = np.isfinite(xyz).all(axis=-1)
    ranges = np.linalg.norm(xyz, axis=-1)
    valid = finite & (ranges > minimum_range)
    rows, columns = np.nonzero(valid)

    selected_ranges = ranges[rows, columns]
    selected_xyz = xyz[rows, columns]
    if len(selected_ranges):
        directions = selected_xyz / selected_ranges[:, None]
    else:
        directions = np.empty((0, 3), dtype=np.float64)

    origins = np.zeros_like(directions)
    rays = np.concatenate((origins, directions), axis=1).astype(np.float32)
    total = decoded.height * decoded.width
    return RayBundle(
        rows=rows,
        columns=columns,
        directions=directions,
        ranges=selected_ranges,
        rays=rays,
        total_points=total,
        invalid_points=total - len(selected_ranges),
    )

