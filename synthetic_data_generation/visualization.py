"""RViz MarkerArray construction from the exact frame-injection result."""

from __future__ import annotations

import copy
import math
from typing import Any

import numpy as np

from synthetic_data_generation.geometry import BuiltGeometry
from synthetic_data_generation.object_injector import FrameInjectionResult
from synthetic_data_generation.scenario import VisualizationConfig


class VisualizationUnavailableError(ImportError):
    pass


def _messages():
    try:
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        from visualization_msgs.msg import Marker, MarkerArray
    except ImportError as exc:
        raise VisualizationUnavailableError(
            "visualization_msgs, geometry_msgs, std_msgs, and builtin_interfaces are required when visualization is enabled"
        ) from exc
    return Duration, Point, ColorRGBA, Marker, MarkerArray


def _duration(value: float, duration_type: Any) -> Any:
    sec = int(math.floor(value))
    nanosec = int(round((value - sec) * 1_000_000_000))
    if nanosec == 1_000_000_000:
        sec += 1; nanosec = 0
    return duration_type(sec=sec, nanosec=nanosec)


def _quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2,1]-rotation[1,2])/s, (rotation[0,2]-rotation[2,0])/s,
                (rotation[1,0]-rotation[0,1])/s, 0.25*s)
    index = int(np.argmax(np.diag(rotation)))
    if index == 0:
        s = math.sqrt(1.0 + rotation[0,0] - rotation[1,1] - rotation[2,2]) * 2.0
        return (0.25*s, (rotation[0,1]+rotation[1,0])/s, (rotation[0,2]+rotation[2,0])/s, (rotation[2,1]-rotation[1,2])/s)
    if index == 1:
        s = math.sqrt(1.0 + rotation[1,1] - rotation[0,0] - rotation[2,2]) * 2.0
        return ((rotation[0,1]+rotation[1,0])/s, 0.25*s, (rotation[1,2]+rotation[2,1])/s, (rotation[0,2]-rotation[2,0])/s)
    s = math.sqrt(1.0 + rotation[2,2] - rotation[0,0] - rotation[1,1]) * 2.0
    return ((rotation[0,2]+rotation[2,0])/s, (rotation[1,2]+rotation[2,1])/s, 0.25*s, (rotation[1,0]-rotation[0,1])/s)


def _set_pose(marker: Any, transform: np.ndarray) -> None:
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, transform[:3, 3])
    qx, qy, qz, qw = _quaternion(transform[:3, :3])
    marker.pose.orientation.x = qx; marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz; marker.pose.orientation.w = qw


def _identity_pose(marker: Any) -> None:
    marker.pose.orientation.w = 1.0


def _point(values: np.ndarray | tuple[float, ...], point_type: Any) -> Any:
    return point_type(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def _base_marker(cloud: Any, geometry: BuiltGeometry, suffix: str, marker_id: int,
                 marker_type: int, lifetime: Any, marker_cls: Any, color_cls: Any) -> Any:
    marker = marker_cls()
    marker.header = copy.deepcopy(cloud.header)
    marker.ns = f"synthetic/{geometry.object_id}/{suffix}"
    marker.id = marker_id
    marker.type = marker_type
    marker.action = marker_cls.ADD
    marker.lifetime = lifetime
    marker.color = color_cls(**dict(zip(("r", "g", "b", "a"), geometry.color_rgba)))
    return marker


def _geometry_marker(cloud: Any, item: BuiltGeometry, lifetime: Any, marker_cls: Any,
                     point_cls: Any, color_cls: Any) -> Any:
    if item.geometry_type == "box":
        marker_type = marker_cls.CUBE
    elif item.geometry_type == "cylinder":
        marker_type = marker_cls.CYLINDER
    elif item.geometry_type == "cable":
        marker_type = marker_cls.LINE_STRIP
    elif item.geometry_type == "human_mesh" and item.mesh_resource_uri:
        marker_type = marker_cls.MESH_RESOURCE
    else:
        marker_type = marker_cls.TRIANGLE_LIST if len(item.triangles) <= 20_000 else marker_cls.CUBE
    marker = _base_marker(cloud, item, "geometry", 0, marker_type, lifetime, marker_cls, color_cls)
    if marker_type in {marker_cls.CUBE, marker_cls.CYLINDER} and not (
            item.geometry_type == "human_mesh" and len(item.triangles) > 20_000):
        _set_pose(marker, item.marker_transform)
        marker.scale.x, marker.scale.y, marker.scale.z = item.marker_scale
    elif marker_type == marker_cls.MESH_RESOURCE:
        _set_pose(marker, item.marker_transform)
        marker.scale.x, marker.scale.y, marker.scale.z = item.marker_scale
        marker.mesh_resource = item.mesh_resource_uri
        marker.mesh_use_embedded_materials = False
    elif marker_type == marker_cls.LINE_STRIP:
        _identity_pose(marker)
        marker.scale.x = item.marker_scale[0]
        marker.points = [_point(point, point_cls) for point in item.control_points_lidar]
    elif marker_type == marker_cls.TRIANGLE_LIST:
        _identity_pose(marker)
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.points = [_point(item.vertices_lidar[index], point_cls) for triangle in item.triangles for index in triangle]
    else:  # oversized mesh fallback: world-frame AABB cube
        _identity_pose(marker)
        minimum, maximum = np.asarray(item.bounds_min), np.asarray(item.bounds_max)
        center, size = (minimum + maximum) / 2.0, maximum - minimum
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, center)
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, size)
    return marker


def _bbox_marker(cloud: Any, item: BuiltGeometry, lifetime: Any, marker_cls: Any,
                 point_cls: Any, color_cls: Any) -> Any:
    marker = _base_marker(cloud, item, "bbox", 3, marker_cls.LINE_LIST, lifetime, marker_cls, color_cls)
    _identity_pose(marker); marker.scale.x = 0.02
    lo, hi = np.asarray(item.bounds_min), np.asarray(item.bounds_max)
    corners = np.array([[x,y,z] for z in (lo[2],hi[2]) for y in (lo[1],hi[1]) for x in (lo[0],hi[0])])
    edges = ((0,1),(0,2),(1,3),(2,3),(4,5),(4,6),(5,7),(6,7),(0,4),(1,5),(2,6),(3,7))
    marker.points = [_point(corners[index], point_cls) for edge in edges for index in edge]
    return marker


def build_marker_array(cloud: Any, geometries: tuple[BuiltGeometry, ...], result: FrameInjectionResult,
                       config: VisualizationConfig) -> Any:
    Duration, Point, ColorRGBA, Marker, MarkerArray = _messages()
    lifetime = _duration(config.marker_lifetime_sec, Duration)
    stats = {item.object_id: item for item in result.object_stats}
    markers: list[Any] = []
    for item in geometries:
        object_stats = stats[item.object_id]
        if config.show_geometry:
            markers.append(_geometry_marker(cloud, item, lifetime, Marker, Point, ColorRGBA))
        if config.show_modified_points:
            marker = _base_marker(cloud, item, "points", 1, Marker.POINTS, lifetime, Marker, ColorRGBA)
            _identity_pose(marker)
            marker.scale.x = config.point_size_m; marker.scale.y = config.point_size_m
            mask = result.winning_object_ids == item.object_id
            marker.points = [_point(point, Point) for point in result.modified_xyz[mask]]
            markers.append(marker)
        if config.show_text:
            marker = _base_marker(cloud, item, "text", 2, Marker.TEXT_VIEW_FACING, lifetime, Marker, ColorRGBA)
            _identity_pose(marker); marker.scale.z = max(0.2, config.point_size_m * 4.0)
            marker.pose.position.x = (item.bounds_min[0] + item.bounds_max[0]) / 2.0
            marker.pose.position.y = (item.bounds_min[1] + item.bounds_max[1]) / 2.0
            marker.pose.position.z = item.bounds_max[2] + marker.scale.z
            marker.text = f"{item.object_id} | {item.class_name} | {item.geometry_type} | modified={object_stats.modified_slot_count}"
            markers.append(marker)
        if config.show_bounding_box:
            markers.append(_bbox_marker(cloud, item, lifetime, Marker, Point, ColorRGBA))
    return MarkerArray(markers=markers)
