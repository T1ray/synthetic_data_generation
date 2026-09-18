"""Strict loaders for legacy schema v1 and multi-object schema v2."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Union

import numpy as np
import yaml


class ScenarioError(ValueError):
    """Raised when a scenario does not conform to a supported schema."""


@dataclass(frozen=True)
class PoseConfig:
    coordinate_system: str
    xyz_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]


@dataclass(frozen=True)
class BoxGeometry:
    dimensions_m: tuple[float, float, float]
    origin: str = "base_center"
    type: str = "box"


@dataclass(frozen=True)
class CylinderGeometry:
    radius_m: float
    height_m: float
    radial_segments: int
    type: str = "cylinder"


@dataclass(frozen=True)
class HumanMeshGeometry:
    path: Path
    source_path: str
    units: str
    scale: tuple[float, float, float]
    origin: str
    mesh_resource_uri: Optional[str]
    type: str = "human_mesh"


@dataclass(frozen=True)
class CableGeometry:
    radius_m: float
    radial_segments: int
    control_points_m: tuple[tuple[float, float, float], ...]
    type: str = "cable"


GeometryConfig = Union[BoxGeometry, CylinderGeometry, HumanMeshGeometry, CableGeometry]


@dataclass(frozen=True)
class ObjectVisualization:
    color_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class VisualizationConfig:
    enabled: bool = False
    marker_topic: str = "/synthetic/markers"
    point_size_m: float = 0.06
    marker_lifetime_sec: float = 0.25
    show_geometry: bool = True
    show_modified_points: bool = True
    show_text: bool = True
    show_bounding_box: bool = True


@dataclass(frozen=True)
class ScenarioObject:
    object_id: str
    class_name: str
    geometry: GeometryConfig
    pose: PoseConfig
    visualization: ObjectVisualization

    @property
    def geometry_type(self) -> str:
        return self.geometry.type

    @property
    def dimensions_m(self) -> tuple[float, float, float]:
        if not isinstance(self.geometry, BoxGeometry):
            raise AttributeError("dimensions_m is only defined for box geometry")
        return self.geometry.dimensions_m

    @property
    def coordinate_system(self) -> str:
        return self.pose.coordinate_system

    @property
    def xyz_m(self) -> tuple[float, float, float]:
        return self.pose.xyz_m

    @property
    def rpy_deg(self) -> tuple[float, float, float]:
        return self.pose.rpy_deg


@dataclass(frozen=True)
class Scenario:
    schema_version: int
    scenario_id: str
    seed: int
    pointcloud_topic: str
    start_index: int
    end_index: int
    objects: tuple[ScenarioObject, ...]
    visualization: VisualizationConfig

    @property
    def object(self) -> ScenarioObject:
        """Compatibility accessor for schema-v1 callers."""
        return self.objects[0]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ScenarioError(f"{path}: expected a mapping")
    return value


def _keys(value: Mapping[str, Any], path: str, required: set[str], optional: set[str] | None = None) -> None:
    optional = optional or set()
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing:
        raise ScenarioError(f"{path}: missing required key(s): {', '.join(missing)}")
    if unknown:
        raise ScenarioError(f"{path}: unsupported key(s): {', '.join(unknown)}")


def _text(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ScenarioError(f"{path}: expected a non-empty string")
    return value


def _integer(value: Any, path: str, *, minimum: int | None = None, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScenarioError(f"{path}: expected an integer")
    if minimum is not None and value < minimum:
        raise ScenarioError(f"{path}: expected a value >= {minimum}")
    if maximum is not None and value > maximum:
        raise ScenarioError(f"{path}: expected a value <= {maximum}")
    return value


def _number(value: Any, path: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{path}: expected a number")
    result = float(value)
    if not math.isfinite(result):
        raise ScenarioError(f"{path}: expected a finite number")
    if positive and result <= 0.0:
        raise ScenarioError(f"{path}: expected a positive number")
    return result


def _vector(value: Any, path: str, length: int, *, positive: bool = False) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ScenarioError(f"{path}: expected exactly {length} numbers")
    return tuple(_number(item, f"{path}[{index}]", positive=positive) for index, item in enumerate(value))


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ScenarioError(f"{path}: expected true or false")
    return value


def _parse_pose(raw: Any, path: str, *, allow_rotation: bool) -> PoseConfig:
    pose = _mapping(raw, path)
    _keys(pose, path, {"coordinate_system", "xyz_m", "rpy_deg"})
    coordinate_system = _text(pose["coordinate_system"], f"{path}.coordinate_system")
    if coordinate_system != "lidar":
        raise ScenarioError(f"{path}.coordinate_system: unsupported value {coordinate_system!r}; expected 'lidar'")
    xyz = _vector(pose["xyz_m"], f"{path}.xyz_m", 3)
    rpy = _vector(pose["rpy_deg"], f"{path}.rpy_deg", 3)
    if not allow_rotation and any(value != 0.0 for value in rpy):
        raise ScenarioError(f"{path}.rpy_deg: rotated boxes require schema_version 2")
    return PoseConfig(coordinate_system, xyz, rpy)  # type: ignore[arg-type]


def _parse_color(raw: Any, path: str) -> ObjectVisualization:
    visual = _mapping(raw, path)
    _keys(visual, path, {"color_rgba"})
    color = _vector(visual["color_rgba"], f"{path}.color_rgba", 4)
    if any(value < 0.0 or value > 1.0 for value in color):
        raise ScenarioError(f"{path}.color_rgba: every component must be in [0, 1]")
    if color[3] <= 0.0:
        raise ScenarioError(f"{path}.color_rgba[3]: alpha must be greater than zero")
    return ObjectVisualization(color)  # type: ignore[arg-type]


def _parse_geometry(raw: Any, path: str, base_dir: Path, *, legacy: bool) -> GeometryConfig:
    geometry = _mapping(raw, path)
    kind = _text(geometry.get("type"), f"{path}.type")
    if kind == "box":
        _keys(geometry, path, {"type", "dimensions_m"})
        dimensions = _vector(geometry["dimensions_m"], f"{path}.dimensions_m", 3, positive=True)
        return BoxGeometry(dimensions, "center" if legacy else "base_center")  # type: ignore[arg-type]
    if legacy:
        raise ScenarioError(f"{path}.type: schema version 1 supports only 'box'")
    if kind == "cylinder":
        _keys(geometry, path, {"type", "radius_m", "height_m", "radial_segments"})
        return CylinderGeometry(
            _number(geometry["radius_m"], f"{path}.radius_m", positive=True),
            _number(geometry["height_m"], f"{path}.height_m", positive=True),
            _integer(geometry["radial_segments"], f"{path}.radial_segments", minimum=8, maximum=256),
        )
    if kind == "human_mesh":
        _keys(geometry, path, {"type", "path", "units", "scale", "origin"}, {"mesh_resource_uri"})
        source_path = _text(geometry["path"], f"{path}.path")
        mesh_path = (base_dir / source_path).resolve() if not Path(source_path).is_absolute() else Path(source_path).resolve()
        if not mesh_path.is_file():
            raise ScenarioError(f"{path}.path: mesh file does not exist: {mesh_path}")
        if mesh_path.suffix.lower() not in {".obj", ".stl"}:
            raise ScenarioError(f"{path}.path: unsupported mesh extension {mesh_path.suffix!r}; expected .obj or .stl")
        units = _text(geometry["units"], f"{path}.units")
        if units not in {"meters", "centimeters", "millimeters"}:
            raise ScenarioError(f"{path}.units: expected meters, centimeters, or millimeters")
        origin = _text(geometry["origin"], f"{path}.origin")
        if origin not in {"base_center", "mesh_origin"}:
            raise ScenarioError(f"{path}.origin: expected base_center or mesh_origin")
        uri = geometry.get("mesh_resource_uri")
        if uri is not None and (not isinstance(uri, str) or not uri.startswith("package://")):
            raise ScenarioError(f"{path}.mesh_resource_uri: expected a package:// URI")
        scale = _vector(geometry["scale"], f"{path}.scale", 3, positive=True)
        return HumanMeshGeometry(mesh_path, source_path, units, scale, origin, uri)  # type: ignore[arg-type]
    if kind == "cable":
        _keys(geometry, path, {"type", "radius_m", "radial_segments", "control_points_m"})
        raw_points = geometry["control_points_m"]
        if not isinstance(raw_points, list) or len(raw_points) < 2:
            raise ScenarioError(f"{path}.control_points_m: expected at least two points")
        points = tuple(_vector(point, f"{path}.control_points_m[{index}]", 3) for index, point in enumerate(raw_points))
        for index in range(1, len(points)):
            if np.linalg.norm(np.asarray(points[index]) - np.asarray(points[index - 1])) <= 1e-9:
                raise ScenarioError(f"{path}.control_points_m[{index}]: consecutive points must be distinct")
        return CableGeometry(
            _number(geometry["radius_m"], f"{path}.radius_m", positive=True),
            _integer(geometry["radial_segments"], f"{path}.radial_segments", minimum=8, maximum=256),
            points,  # type: ignore[arg-type]
        )
    raise ScenarioError(f"{path}.type: unsupported geometry type {kind!r}")


def _parse_visualization(raw: Any) -> VisualizationConfig:
    path = "visualization"
    value = _mapping(raw, path)
    required = {"enabled", "marker_topic", "point_size_m", "marker_lifetime_sec", "show_geometry", "show_modified_points", "show_text", "show_bounding_box"}
    _keys(value, path, required)
    enabled = _boolean(value["enabled"], f"{path}.enabled")
    topic = _text(value["marker_topic"], f"{path}.marker_topic")
    point_size = _number(value["point_size_m"], f"{path}.point_size_m", positive=True)
    lifetime = _number(value["marker_lifetime_sec"], f"{path}.marker_lifetime_sec", positive=True)
    return VisualizationConfig(enabled, topic, point_size, lifetime,
        _boolean(value["show_geometry"], f"{path}.show_geometry"),
        _boolean(value["show_modified_points"], f"{path}.show_modified_points"),
        _boolean(value["show_text"], f"{path}.show_text"),
        _boolean(value["show_bounding_box"], f"{path}.show_bounding_box"))


def parse_scenario(document: Any, *, base_dir: Path | None = None) -> Scenario:
    root = _mapping(document, "scenario")
    version = _integer(root.get("schema_version"), "schema_version")
    if version not in {1, 2}:
        raise ScenarioError(f"schema_version: unsupported value {version}; expected 1 or 2")
    required = {"schema_version", "scenario_id", "seed", "source", "frames", "objects", "sensor_effects"}
    optional = {"visualization"} if version == 2 else set()
    _keys(root, "scenario", required, optional)
    scenario_id = _text(root["scenario_id"], "scenario_id")
    seed = _integer(root["seed"], "seed", minimum=0, maximum=0xFFFFFFFF)
    source = _mapping(root["source"], "source")
    _keys(source, "source", {"pointcloud_topic"})
    topic = _text(source["pointcloud_topic"], "source.pointcloud_topic")
    frames = _mapping(root["frames"], "frames")
    _keys(frames, "frames", {"start_index", "end_index"})
    start = _integer(frames["start_index"], "frames.start_index", minimum=0)
    end = _integer(frames["end_index"], "frames.end_index", minimum=0)
    if start > end:
        raise ScenarioError("frames: start_index must be less than or equal to end_index")

    raw_objects = root["objects"]
    if not isinstance(raw_objects, list) or not raw_objects:
        raise ScenarioError("objects: expected one or more objects")
    objects: list[ScenarioObject] = []
    ids: set[str] = set()
    for index, raw in enumerate(raw_objects):
        path = f"objects[{index}]"
        item = _mapping(raw, path)
        required_object = {"id", "class_name", "geometry", "pose"}
        optional_object = {"visualization"} if version == 2 else set()
        _keys(item, path, required_object, optional_object)
        object_id = _text(item["id"], f"{path}.id")
        if object_id in ids:
            raise ScenarioError(f"{path}.id: duplicate object id {object_id!r}")
        ids.add(object_id)
        if version == 1 and index > 0:
            raise ScenarioError(f"objects: schema version 1 requires exactly one object; got {len(raw_objects)}")
        color = (_parse_color(item["visualization"], f"{path}.visualization")
                 if version == 2 and "visualization" in item
                 else ObjectVisualization((1.0, 0.2, 0.1, 0.8)))
        objects.append(ScenarioObject(
            object_id,
            _text(item["class_name"], f"{path}.class_name"),
            _parse_geometry(item["geometry"], f"{path}.geometry", base_dir or Path.cwd(), legacy=version == 1),
            _parse_pose(item["pose"], f"{path}.pose", allow_rotation=version == 2),
            color,
        ))

    effects = _mapping(root["sensor_effects"], "sensor_effects")
    effect_keys = {"range_noise", "dropout", "modify_intensity"}
    _keys(effects, "sensor_effects", effect_keys)
    for name in sorted(effect_keys):
        if effects[name] is not False:
            raise ScenarioError(f"sensor_effects.{name}: only false is supported")
    visualization = (_parse_visualization(root["visualization"])
                     if version == 2 and "visualization" in root else VisualizationConfig())
    if visualization.enabled and visualization.marker_topic == topic:
        raise ScenarioError("visualization.marker_topic must differ from source.pointcloud_topic")
    return Scenario(version, scenario_id, seed, topic, start, end, tuple(objects), visualization)


def load_scenario(path: Path | str) -> Scenario:
    try:
        scenario_path = Path(path).expanduser().resolve(strict=True)
        with scenario_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as exc:
        raise ScenarioError(f"scenario file: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ScenarioError(f"scenario YAML: {exc}") from exc
    return parse_scenario(document, base_dir=scenario_path.parent)
