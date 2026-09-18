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
class MaterialConfig:
    reflectivity: float = 1.0
    return_probability_scale: float = 1.0


@dataclass(frozen=True)
class ZeroSlotRecoveryConfig:
    enabled: bool = False
    ring_field: str = "ring"
    timestamp_field: str = "timestamp"
    timestamp_unit: str = "seconds"
    min_valid_samples_per_ring: int = 8
    interpolation: str = "linear"
    allow_extrapolation: bool = False
    max_angular_error_deg: float = 0.25


@dataclass(frozen=True)
class RangeNoiseConfig:
    enabled: bool = False
    model: str = "gaussian"
    base_sigma_m: float = 0.0
    sigma_per_meter: float = 0.0
    incidence_sigma_scale: float = 0.0
    min_range_m: float = 0.0
    max_resample_attempts: int = 8


@dataclass(frozen=True)
class DropoutConfig:
    enabled: bool = False
    base_return_probability: float = 1.0
    distance_reference_m: float = 30.0
    distance_exponent: float = 1.0
    incidence_exponent: float = 1.0
    min_return_probability: float = 0.0


@dataclass(frozen=True)
class IntensityConfig:
    enabled: bool = False
    model: str = "empirical"
    field_name: str = "intensity"
    range_bin_m: float = 5.0
    min_samples_per_bin: int = 32
    incidence_exponent: float = 1.0
    additive_sigma: float = 0.0
    fallback: str = "preserve_original"
    on_missing: str = "error"
    constant: float = 0.0


@dataclass(frozen=True)
class SensorEffectsConfig:
    no_return_encoding: str = "zero_xyz"
    range_noise: RangeNoiseConfig = RangeNoiseConfig()
    dropout: DropoutConfig = DropoutConfig()
    intensity: IntensityConfig = IntensityConfig()


@dataclass(frozen=True)
class LidarPlacement:
    xyz_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    type: str = "lidar_relative"


@dataclass(frozen=True)
class TrackPlacement:
    longitudinal_m: float
    lateral_m: float
    height_m: float
    rpy_track_deg: tuple[float, float, float]
    type: str = "track_relative"


PlacementConfig = Union[LidarPlacement, TrackPlacement]


@dataclass(frozen=True)
class MotionConfig:
    type: str = "static"
    longitudinal_velocity_mps: float = 0.0
    lateral_velocity_mps: float = 0.0
    vertical_velocity_mps: float = 0.0
    yaw_rate_deg_s: float = 0.0


@dataclass(frozen=True)
class TemporalConfig:
    start_index: int
    end_index: int
    motion: MotionConfig


@dataclass(frozen=True)
class TrackConfig:
    reference_frame: str
    points_m: tuple[tuple[float, float, float], ...]
    up_hint: tuple[float, float, float]


@dataclass(frozen=True)
class EgoMotionConfig:
    source: str
    path: Path
    reference_frame: str
    lidar_frame: str
    timestamp_unit: str
    max_interpolation_gap_ms: float


@dataclass(frozen=True)
class ScenarioObject:
    object_id: str
    class_name: str
    geometry: GeometryConfig
    pose: Optional[PoseConfig]
    visualization: ObjectVisualization
    placement: Optional[PlacementConfig] = None
    temporal: Optional[TemporalConfig] = None
    material: MaterialConfig = MaterialConfig()

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
        return self.pose.coordinate_system if self.pose is not None else self.placement.type

    @property
    def xyz_m(self) -> tuple[float, float, float]:
        if self.pose is None:
            raise AttributeError("track-relative object has no fixed lidar xyz_m")
        return self.pose.xyz_m

    @property
    def rpy_deg(self) -> tuple[float, float, float]:
        if self.pose is None:
            raise AttributeError("track-relative object has no fixed lidar rpy_deg")
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
    zero_slot_recovery: ZeroSlotRecoveryConfig = ZeroSlotRecoveryConfig()
    sensor_effects: SensorEffectsConfig = SensorEffectsConfig()
    track: Optional[TrackConfig] = None
    ego_motion: Optional[EgoMotionConfig] = None

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
    required = {"enabled", "marker_topic", "point_size_m", "marker_lifetime_sec"}
    optional = {"show_geometry", "show_modified_points", "show_text", "show_bounding_box"}
    _keys(value, path, required, optional)
    enabled = _boolean(value["enabled"], f"{path}.enabled")
    topic = _text(value["marker_topic"], f"{path}.marker_topic")
    point_size = _number(value["point_size_m"], f"{path}.point_size_m", positive=True)
    lifetime = _number(value["marker_lifetime_sec"], f"{path}.marker_lifetime_sec", positive=True)
    return VisualizationConfig(enabled, topic, point_size, lifetime,
        _boolean(value.get("show_geometry", True), f"{path}.show_geometry"),
        _boolean(value.get("show_modified_points", True), f"{path}.show_modified_points"),
        _boolean(value.get("show_text", True), f"{path}.show_text"),
        _boolean(value.get("show_bounding_box", True), f"{path}.show_bounding_box"))


def _probability(value: Any, path: str) -> float:
    result = _number(value, path)
    if result < 0.0 or result > 1.0:
        raise ScenarioError(f"{path}: expected a value in [0, 1]")
    return result


def _parse_zero_recovery(raw: Any) -> ZeroSlotRecoveryConfig:
    path = "zero_slot_recovery"; value = _mapping(raw, path)
    keys = {"enabled", "ring_field", "timestamp_field", "timestamp_unit", "min_valid_samples_per_ring", "interpolation", "allow_extrapolation", "max_angular_error_deg"}
    _keys(value, path, keys)
    unit = _text(value["timestamp_unit"], f"{path}.timestamp_unit")
    units = {"seconds", "milliseconds", "microseconds", "nanoseconds", "relative_ticks"}
    if unit == "auto":
        raise ScenarioError(f"{path}.timestamp_unit: auto is ambiguous; choose an explicit unit")
    if unit not in units:
        raise ScenarioError(f"{path}.timestamp_unit: unsupported value {unit!r}")
    interpolation = _text(value["interpolation"], f"{path}.interpolation")
    if interpolation != "linear":
        raise ScenarioError(f"{path}.interpolation: only linear is supported")
    return ZeroSlotRecoveryConfig(
        _boolean(value["enabled"], f"{path}.enabled"),
        _text(value["ring_field"], f"{path}.ring_field"),
        _text(value["timestamp_field"], f"{path}.timestamp_field"), unit,
        _integer(value["min_valid_samples_per_ring"], f"{path}.min_valid_samples_per_ring", minimum=2),
        interpolation, _boolean(value["allow_extrapolation"], f"{path}.allow_extrapolation"),
        _number(value["max_angular_error_deg"], f"{path}.max_angular_error_deg", positive=True))


def _parse_sensor_effects_v3(raw: Any) -> SensorEffectsConfig:
    root = _mapping(raw, "sensor_effects")
    _keys(root, "sensor_effects", {"no_return_encoding", "range_noise", "dropout", "intensity"})
    encoding = _text(root["no_return_encoding"], "sensor_effects.no_return_encoding")
    if encoding != "zero_xyz":
        raise ScenarioError("sensor_effects.no_return_encoding: only zero_xyz is supported")
    rn = _mapping(root["range_noise"], "sensor_effects.range_noise")
    rn_keys = {"enabled", "model", "base_sigma_m", "sigma_per_meter", "incidence_sigma_scale", "min_range_m", "max_resample_attempts"}
    _keys(rn, "sensor_effects.range_noise", rn_keys)
    if rn["model"] != "gaussian":
        raise ScenarioError("sensor_effects.range_noise.model: only gaussian is supported")
    range_noise = RangeNoiseConfig(_boolean(rn["enabled"], "sensor_effects.range_noise.enabled"), "gaussian",
        _number(rn["base_sigma_m"], "sensor_effects.range_noise.base_sigma_m"),
        _number(rn["sigma_per_meter"], "sensor_effects.range_noise.sigma_per_meter"),
        _number(rn["incidence_sigma_scale"], "sensor_effects.range_noise.incidence_sigma_scale"),
        _number(rn["min_range_m"], "sensor_effects.range_noise.min_range_m", positive=True),
        _integer(rn["max_resample_attempts"], "sensor_effects.range_noise.max_resample_attempts", minimum=0, maximum=100))
    if min(range_noise.base_sigma_m, range_noise.sigma_per_meter, range_noise.incidence_sigma_scale) < 0:
        raise ScenarioError("sensor_effects.range_noise: sigma terms must be non-negative")
    dr = _mapping(root["dropout"], "sensor_effects.dropout")
    dr_keys = {"enabled", "base_return_probability", "distance_reference_m", "distance_exponent", "incidence_exponent", "min_return_probability"}
    _keys(dr, "sensor_effects.dropout", dr_keys)
    dropout = DropoutConfig(_boolean(dr["enabled"], "sensor_effects.dropout.enabled"),
        _probability(dr["base_return_probability"], "sensor_effects.dropout.base_return_probability"),
        _number(dr["distance_reference_m"], "sensor_effects.dropout.distance_reference_m", positive=True),
        _number(dr["distance_exponent"], "sensor_effects.dropout.distance_exponent"),
        _number(dr["incidence_exponent"], "sensor_effects.dropout.incidence_exponent"),
        _probability(dr["min_return_probability"], "sensor_effects.dropout.min_return_probability"))
    if dropout.distance_exponent < 0 or dropout.incidence_exponent < 0:
        raise ScenarioError("sensor_effects.dropout: exponents must be non-negative")
    it = _mapping(root["intensity"], "sensor_effects.intensity")
    it_required = {"enabled", "model", "field_name", "range_bin_m", "min_samples_per_bin", "incidence_exponent", "additive_sigma", "fallback", "on_missing"}
    _keys(it, "sensor_effects.intensity", it_required, {"constant"})
    if it["model"] != "empirical":
        raise ScenarioError("sensor_effects.intensity.model: only empirical is supported")
    fallback = _text(it["fallback"], "sensor_effects.intensity.fallback")
    if fallback not in {"preserve_original", "constant"}:
        raise ScenarioError("sensor_effects.intensity.fallback: expected preserve_original or constant")
    on_missing = _text(it["on_missing"], "sensor_effects.intensity.on_missing")
    if on_missing not in {"error", "skip_with_warning"}:
        raise ScenarioError("sensor_effects.intensity.on_missing: expected error or skip_with_warning")
    intensity = IntensityConfig(_boolean(it["enabled"], "sensor_effects.intensity.enabled"), "empirical",
        _text(it["field_name"], "sensor_effects.intensity.field_name"),
        _number(it["range_bin_m"], "sensor_effects.intensity.range_bin_m", positive=True),
        _integer(it["min_samples_per_bin"], "sensor_effects.intensity.min_samples_per_bin", minimum=1),
        _number(it["incidence_exponent"], "sensor_effects.intensity.incidence_exponent"),
        _number(it["additive_sigma"], "sensor_effects.intensity.additive_sigma"), fallback, on_missing,
        _number(it.get("constant", 0.0), "sensor_effects.intensity.constant"))
    if intensity.incidence_exponent < 0 or intensity.additive_sigma < 0:
        raise ScenarioError("sensor_effects.intensity: exponent and sigma must be non-negative")
    return SensorEffectsConfig(encoding, range_noise, dropout, intensity)


def _parse_material(raw: Any, path: str) -> MaterialConfig:
    value = _mapping(raw, path); _keys(value, path, {"reflectivity", "return_probability_scale"})
    return MaterialConfig(_probability(value["reflectivity"], f"{path}.reflectivity"),
                          _probability(value["return_probability_scale"], f"{path}.return_probability_scale"))


def _parse_placement(raw: Any, path: str) -> PlacementConfig:
    value = _mapping(raw, path); kind = _text(value.get("type"), f"{path}.type")
    if kind == "lidar_relative":
        _keys(value, path, {"type", "xyz_m", "rpy_deg"})
        return LidarPlacement(_vector(value["xyz_m"], f"{path}.xyz_m", 3), _vector(value["rpy_deg"], f"{path}.rpy_deg", 3))  # type: ignore[arg-type]
    if kind == "track_relative":
        _keys(value, path, {"type", "longitudinal_m", "lateral_m", "height_m", "rpy_track_deg"})
        return TrackPlacement(_number(value["longitudinal_m"], f"{path}.longitudinal_m"),
            _number(value["lateral_m"], f"{path}.lateral_m"), _number(value["height_m"], f"{path}.height_m"),
            _vector(value["rpy_track_deg"], f"{path}.rpy_track_deg", 3))  # type: ignore[arg-type]
    raise ScenarioError(f"{path}.type: expected lidar_relative or track_relative")


def _parse_temporal(raw: Any, path: str) -> TemporalConfig:
    value = _mapping(raw, path); _keys(value, path, {"active_frames", "motion"})
    active = _mapping(value["active_frames"], f"{path}.active_frames")
    _keys(active, f"{path}.active_frames", {"start_index", "end_index"})
    start = _integer(active["start_index"], f"{path}.active_frames.start_index", minimum=0)
    end = _integer(active["end_index"], f"{path}.active_frames.end_index", minimum=0)
    if start > end: raise ScenarioError(f"{path}.active_frames: start_index must not exceed end_index")
    motion_raw = _mapping(value["motion"], f"{path}.motion")
    kind = _text(motion_raw.get("type"), f"{path}.motion.type")
    if kind == "static":
        _keys(motion_raw, f"{path}.motion", {"type"}); motion = MotionConfig()
    elif kind == "constant_track_velocity":
        keys = {"type", "longitudinal_velocity_mps", "lateral_velocity_mps", "vertical_velocity_mps", "yaw_rate_deg_s"}
        _keys(motion_raw, f"{path}.motion", keys)
        motion = MotionConfig(kind, *[_number(motion_raw[name], f"{path}.motion.{name}") for name in (
            "longitudinal_velocity_mps", "lateral_velocity_mps", "vertical_velocity_mps", "yaw_rate_deg_s")])
    else: raise ScenarioError(f"{path}.motion.type: unsupported value {kind!r}")
    return TemporalConfig(start, end, motion)


def _parse_track(raw: Any) -> TrackConfig:
    value = _mapping(raw, "track"); _keys(value, "track", {"reference_frame", "model", "up_hint"})
    model = _mapping(value["model"], "track.model"); _keys(model, "track.model", {"type", "points_m"})
    if model["type"] != "polyline": raise ScenarioError("track.model.type: only polyline is supported")
    raw_points = model["points_m"]
    if not isinstance(raw_points, list) or len(raw_points) < 2: raise ScenarioError("track.model.points_m: expected at least two points")
    points = tuple(_vector(point, f"track.model.points_m[{index}]", 3) for index, point in enumerate(raw_points))
    for index in range(1, len(points)):
        if np.linalg.norm(np.asarray(points[index])-np.asarray(points[index-1])) <= 1e-9:
            raise ScenarioError(f"track.model.points_m[{index}]: zero-length segment")
    up = _vector(value["up_hint"], "track.up_hint", 3)
    if np.linalg.norm(up) <= 1e-9: raise ScenarioError("track.up_hint: vector must be non-zero")
    return TrackConfig(_text(value["reference_frame"], "track.reference_frame"), points, up)  # type: ignore[arg-type]


def _parse_ego_motion(raw: Any, base_dir: Path) -> EgoMotionConfig:
    value = _mapping(raw, "ego_motion")
    keys = {"source", "path", "reference_frame", "lidar_frame", "timestamp_unit", "max_interpolation_gap_ms"}
    _keys(value, "ego_motion", keys)
    if value["source"] != "trajectory_csv": raise ScenarioError("ego_motion.source: only trajectory_csv is supported")
    source_path = _text(value["path"], "ego_motion.path")
    path = (base_dir/source_path).resolve() if not Path(source_path).is_absolute() else Path(source_path).resolve()
    if not path.is_file(): raise ScenarioError(f"ego_motion.path: trajectory file does not exist: {path}")
    if value["timestamp_unit"] != "nanoseconds": raise ScenarioError("ego_motion.timestamp_unit: only nanoseconds is supported")
    return EgoMotionConfig("trajectory_csv", path, _text(value["reference_frame"], "ego_motion.reference_frame"),
        _text(value["lidar_frame"], "ego_motion.lidar_frame"), "nanoseconds",
        _number(value["max_interpolation_gap_ms"], "ego_motion.max_interpolation_gap_ms", positive=True))


def parse_scenario(document: Any, *, base_dir: Path | None = None) -> Scenario:
    root = _mapping(document, "scenario")
    version = _integer(root.get("schema_version"), "schema_version")
    if version not in {1, 2, 3}:
        raise ScenarioError(f"schema_version: unsupported value {version}; expected 1, 2, or 3")
    required = {"schema_version", "scenario_id", "seed", "source", "frames", "objects", "sensor_effects"}
    if version == 3:
        required |= {"visualization", "zero_slot_recovery"}
        optional = {"track", "ego_motion"}
    else:
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
        if version == 3:
            required_object = {"id", "class_name", "geometry", "placement", "temporal", "material", "visualization"}
            optional_object = set()
        else:
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
                 if version >= 2 and "visualization" in item
                 else ObjectVisualization((1.0, 0.2, 0.1, 0.8)))
        pose = (_parse_pose(item["pose"], f"{path}.pose", allow_rotation=version == 2)
                if version < 3 else None)
        placement = _parse_placement(item["placement"], f"{path}.placement") if version == 3 else None
        temporal = _parse_temporal(item["temporal"], f"{path}.temporal") if version == 3 else None
        if temporal is not None and (temporal.start_index < start or temporal.end_index > end):
            raise ScenarioError(f"{path}.temporal.active_frames must be inside scenario frames [{start}, {end}]")
        if isinstance(placement, LidarPlacement) and temporal.motion.type != "static":
            raise ScenarioError(f"{path}.temporal.motion: lidar_relative placement supports only static motion")
        material = _parse_material(item["material"], f"{path}.material") if version == 3 else MaterialConfig()
        objects.append(ScenarioObject(
            object_id,
            _text(item["class_name"], f"{path}.class_name"),
            _parse_geometry(item["geometry"], f"{path}.geometry", base_dir or Path.cwd(), legacy=version == 1),
            pose, color, placement, temporal, material,
        ))

    if version == 3:
        sensor_effects = _parse_sensor_effects_v3(root["sensor_effects"])
        zero_recovery = _parse_zero_recovery(root["zero_slot_recovery"])
        track = _parse_track(root["track"]) if "track" in root else None
        ego_motion = _parse_ego_motion(root["ego_motion"], base_dir or Path.cwd()) if "ego_motion" in root else None
        if any(isinstance(item.placement, TrackPlacement) for item in objects):
            if track is None or ego_motion is None:
                raise ScenarioError("track-relative objects require both track and ego_motion")
            if track.reference_frame != ego_motion.reference_frame:
                raise ScenarioError("track.reference_frame must match ego_motion.reference_frame")
    else:
        effects = _mapping(root["sensor_effects"], "sensor_effects")
        effect_keys = {"range_noise", "dropout", "modify_intensity"}
        _keys(effects, "sensor_effects", effect_keys)
        for name in sorted(effect_keys):
            if effects[name] is not False:
                raise ScenarioError(f"sensor_effects.{name}: only false is supported")
        sensor_effects = SensorEffectsConfig()
        zero_recovery = ZeroSlotRecoveryConfig()
        track = None; ego_motion = None
    visualization = (_parse_visualization(root["visualization"])
                     if version >= 2 and "visualization" in root else VisualizationConfig())
    if visualization.enabled and visualization.marker_topic == topic:
        raise ScenarioError("visualization.marker_topic must differ from source.pointcloud_topic")
    return Scenario(version, scenario_id, seed, topic, start, end, tuple(objects), visualization,
                    zero_recovery, sensor_effects, track, ego_motion)


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
