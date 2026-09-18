"""Unified deterministic geometry construction in the LiDAR frame."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Optional

import numpy as np

from synthetic_data_generation.scenario import (
    BoxGeometry, CableGeometry, CylinderGeometry, HumanMeshGeometry,
    PoseConfig, ScenarioObject,
)


class GeometryError(ValueError):
    """Raised when geometry cannot be built safely."""


@dataclass(frozen=True)
class BuiltGeometry:
    object_id: str
    class_name: str
    geometry_type: str
    vertices_lidar: np.ndarray
    triangles: np.ndarray
    pose_transform: np.ndarray
    marker_transform: np.ndarray
    marker_scale: tuple[float, float, float]
    bounds_min: tuple[float, float, float]
    bounds_max: tuple[float, float, float]
    color_rgba: tuple[float, float, float, float]
    control_points_lidar: Optional[np.ndarray] = None
    mesh_resource_uri: Optional[str] = None
    mesh_path: Optional[str] = None
    mesh_sha256: Optional[str] = None


def pose_transform(pose: PoseConfig) -> np.ndarray:
    """Return T with R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = np.radians(np.asarray(pose.rpy_deg, dtype=np.float64))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=np.float64)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=np.float64)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = pose.xyz_m
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    homogeneous = np.column_stack((points, np.ones(len(points), dtype=np.float64)))
    return (homogeneous @ transform.T)[:, :3]


def _combine(parts: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[np.ndarray] = []
    triangles: list[np.ndarray] = []
    offset = 0
    for part_vertices, part_triangles in parts:
        vertices.append(part_vertices)
        triangles.append(part_triangles + offset)
        offset += len(part_vertices)
    return np.vstack(vertices), np.vstack(triangles)


def _box_mesh(size: tuple[float, float, float], origin: str) -> tuple[np.ndarray, np.ndarray]:
    sx, sy, sz = size
    z0, z1 = ((-sz / 2.0, sz / 2.0) if origin == "center" else (0.0, sz))
    vertices = np.array([
        [-sx/2, -sy/2, z0], [sx/2, -sy/2, z0], [sx/2, sy/2, z0], [-sx/2, sy/2, z0],
        [-sx/2, -sy/2, z1], [sx/2, -sy/2, z1], [sx/2, sy/2, z1], [-sx/2, sy/2, z1],
    ], dtype=np.float64)
    triangles = np.array([
        [0,2,1],[0,3,2],[4,5,6],[4,6,7], [0,1,5],[0,5,4],
        [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7],
    ], dtype=np.int32)
    return vertices, triangles


def _cylinder_mesh(radius: float, height: float, segments: int) -> tuple[np.ndarray, np.ndarray]:
    angles = np.arange(segments, dtype=np.float64) * (2.0 * math.pi / segments)
    ring = np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))
    vertices = np.vstack((
        np.column_stack((ring, np.zeros(segments))),
        np.column_stack((ring, np.full(segments, height))),
        [[0.0, 0.0, 0.0], [0.0, 0.0, height]],
    ))
    bottom, top = 2 * segments, 2 * segments + 1
    faces: list[list[int]] = []
    for i in range(segments):
        j = (i + 1) % segments
        faces.extend(([i, j, segments+j], [i, segments+j, segments+i], [bottom, j, i], [top, segments+i, segments+j]))
    return vertices, np.asarray(faces, dtype=np.int32)


def _align_z(direction: np.ndarray) -> np.ndarray:
    direction = direction / np.linalg.norm(direction)
    z = np.array((0.0, 0.0, 1.0))
    cross = np.cross(z, direction)
    dot = float(np.dot(z, direction))
    if np.linalg.norm(cross) < 1e-12:
        rotation = np.eye(3)
        if dot < 0:
            rotation = np.diag((1.0, -1.0, -1.0))
        return rotation
    skew = np.array(((0, -cross[2], cross[1]), (cross[2], 0, -cross[0]), (-cross[1], cross[0], 0)))
    return np.eye(3) + skew + skew @ skew * ((1.0 - dot) / np.dot(cross, cross))


def _segment_cylinder(start: np.ndarray, end: np.ndarray, radius: float, segments: int) -> tuple[np.ndarray, np.ndarray]:
    delta = end - start
    vertices, triangles = _cylinder_mesh(radius, float(np.linalg.norm(delta)), segments)
    vertices = vertices @ _align_z(delta).T + start
    return vertices, triangles


def _uv_sphere(center: np.ndarray, radius: float, segments: int) -> tuple[np.ndarray, np.ndarray]:
    rings = max(4, segments // 2)
    vertices = [center + np.array((0.0, 0.0, radius))]
    for r in range(1, rings):
        phi = math.pi * r / rings
        for s in range(segments):
            theta = 2.0 * math.pi * s / segments
            vertices.append(center + radius * np.array((math.sin(phi)*math.cos(theta), math.sin(phi)*math.sin(theta), math.cos(phi))))
    vertices.append(center + np.array((0.0, 0.0, -radius)))
    north, south = 0, len(vertices) - 1
    faces: list[list[int]] = []
    for s in range(segments):
        j = (s + 1) % segments
        faces.append([north, 1+j, 1+s])
        for r in range(rings - 2):
            a = 1 + r*segments + s; b = 1 + r*segments + j
            c = 1 + (r+1)*segments + s; d = 1 + (r+1)*segments + j
            faces.extend(([a,b,d],[a,d,c]))
        last = 1 + (rings-2)*segments
        faces.append([south, last+s, last+j])
    return np.asarray(vertices), np.asarray(faces, dtype=np.int32)


def _load_human_mesh(config: HumanMeshGeometry) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    try:
        import trimesh
    except ImportError as exc:
        raise ImportError("Trimesh is required for human_mesh geometry") from exc
    try:
        loaded = trimesh.load(str(config.path), force="scene", process=False)
        if isinstance(loaded, trimesh.Scene):
            meshes = loaded.dump(concatenate=False)
            meshes = [mesh for mesh in meshes if isinstance(mesh, trimesh.Trimesh)]
            if not meshes:
                raise GeometryError("mesh scene contains no triangular meshes")
            mesh = trimesh.util.concatenate(meshes)
        elif isinstance(loaded, trimesh.Trimesh):
            mesh = loaded
        else:
            raise GeometryError("mesh file did not contain a triangular surface")
    except GeometryError:
        raise
    except Exception as exc:
        raise GeometryError(f"failed to load mesh {config.path}: {exc}") from exc
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    triangles = np.asarray(mesh.faces, dtype=np.int64)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices):
        raise GeometryError("mesh has no vertices")
    if triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles):
        raise GeometryError("mesh has no triangular faces")
    if not np.isfinite(vertices).all() or triangles.min() < 0 or triangles.max() >= len(vertices):
        raise GeometryError("mesh contains invalid vertices or face indices")
    unit_scale = {"meters": 1.0, "centimeters": 0.01, "millimeters": 0.001}[config.units]
    scale = np.asarray(config.scale) * unit_scale
    vertices = vertices * scale
    normalization = np.eye(4)
    if config.origin == "base_center":
        minimum, maximum = vertices.min(axis=0), vertices.max(axis=0)
        translation = np.array((-(minimum[0]+maximum[0])/2.0, -(minimum[1]+maximum[1])/2.0, -minimum[2]))
        vertices += translation
        normalization[:3, 3] = translation
    if np.any(vertices.max(axis=0) - vertices.min(axis=0) <= 0.0):
        raise GeometryError("mesh has zero extent after units, scale, and normalization")
    sha = hashlib.sha256(config.path.read_bytes()).hexdigest()
    return vertices, triangles.astype(np.int32), normalization, sha


def build_geometry(obj: ScenarioObject) -> BuiltGeometry:
    transform = pose_transform(obj.pose)
    geometry = obj.geometry
    marker_transform = transform.copy()
    marker_scale = (1.0, 1.0, 1.0)
    control_points = None
    mesh_path = mesh_sha = None
    mesh_uri = None

    if isinstance(geometry, BoxGeometry):
        local_vertices, triangles = _box_mesh(geometry.dimensions_m, geometry.origin)
        marker_scale = geometry.dimensions_m
        if geometry.origin == "base_center":
            marker_transform = transform.copy()
            marker_transform[:3, 3] = transform_points(np.array([[0.0, 0.0, geometry.dimensions_m[2]/2.0]]), transform)[0]
    elif isinstance(geometry, CylinderGeometry):
        local_vertices, triangles = _cylinder_mesh(geometry.radius_m, geometry.height_m, geometry.radial_segments)
        marker_scale = (2.0 * geometry.radius_m, 2.0 * geometry.radius_m, geometry.height_m)
        marker_transform = transform.copy()
        marker_transform[:3, 3] = transform_points(np.array([[0.0, 0.0, geometry.height_m/2.0]]), transform)[0]
    elif isinstance(geometry, CableGeometry):
        points = np.asarray(geometry.control_points_m, dtype=np.float64)
        parts = [_segment_cylinder(points[i], points[i+1], geometry.radius_m, geometry.radial_segments) for i in range(len(points)-1)]
        parts.extend(_uv_sphere(point, geometry.radius_m, geometry.radial_segments) for point in points)
        local_vertices, triangles = _combine(parts)
        control_points = transform_points(points, transform)
        marker_scale = (2.0 * geometry.radius_m,) * 3
    elif isinstance(geometry, HumanMeshGeometry):
        local_vertices, triangles, normalization, mesh_sha = _load_human_mesh(geometry)
        mesh_path = geometry.source_path
        mesh_uri = geometry.mesh_resource_uri
        marker_transform = transform @ normalization
        unit_scale = {"meters": 1.0, "centimeters": 0.01, "millimeters": 0.001}[geometry.units]
        marker_scale = tuple(unit_scale * value for value in geometry.scale)
    else:  # pragma: no cover - type union is exhaustive
        raise GeometryError(f"unsupported geometry config: {type(geometry).__name__}")

    world_vertices = transform_points(local_vertices, transform)
    if not np.isfinite(world_vertices).all() or not len(triangles):
        raise GeometryError(f"{obj.object_id}: built geometry is invalid")
    minimum, maximum = world_vertices.min(axis=0), world_vertices.max(axis=0)
    return BuiltGeometry(
        obj.object_id, obj.class_name, obj.geometry_type,
        world_vertices, triangles, transform, marker_transform, marker_scale,
        tuple(float(v) for v in minimum), tuple(float(v) for v in maximum),
        obj.visualization.color_rgba, control_points, mesh_uri, mesh_path, mesh_sha,
    )


def build_geometries(objects: tuple[ScenarioObject, ...]) -> tuple[BuiltGeometry, ...]:
    return tuple(build_geometry(obj) for obj in objects)
