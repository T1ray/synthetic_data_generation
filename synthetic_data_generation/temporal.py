"""Explicit ego trajectory and arc-length track-relative object poses."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

from synthetic_data_generation.geometry import BuiltGeometry, build_geometry, pose_transform, transform_built_geometry
from synthetic_data_generation.scenario import (
    EgoMotionConfig, LidarPlacement, PoseConfig, Scenario, ScenarioObject,
    TrackConfig, TrackPlacement,
)


class TemporalError(ValueError):
    pass


class PoseProvider(Protocol):
    def pose_at(self, timestamp_ns: int) -> np.ndarray: ...


@dataclass(frozen=True)
class TrackFrame:
    center: np.ndarray
    tangent: np.ndarray
    lateral: np.ndarray
    up: np.ndarray


class PolylineTrack:
    def __init__(self, config: TrackConfig) -> None:
        self.points = np.asarray(config.points_m, dtype=np.float64)
        self.up_hint = np.asarray(config.up_hint, dtype=np.float64)
        segments = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(segments, axis=1)
        if np.any(self.lengths <= 1e-9):
            raise TemporalError("track polyline contains a zero-length segment")
        self.directions = segments / self.lengths[:, None]
        self.cumulative = np.concatenate(([0.0], np.cumsum(self.lengths)))

    @property
    def length_m(self) -> float:
        return float(self.cumulative[-1])

    def frame_at(self, s_m: float) -> TrackFrame:
        if not np.isfinite(s_m) or s_m < 0.0 or s_m > self.length_m:
            raise TemporalError(f"track arc length {s_m} is outside [0, {self.length_m}]")
        index = min(int(np.searchsorted(self.cumulative, s_m, side="right") - 1), len(self.lengths) - 1)
        fraction = (s_m - self.cumulative[index]) / self.lengths[index]
        center = self.points[index] + fraction * (self.points[index + 1] - self.points[index])
        tangent = self.directions[index]
        lateral = np.cross(self.up_hint, tangent)
        norm = np.linalg.norm(lateral)
        if norm <= 1e-9:
            raise TemporalError("track tangent is parallel to up_hint")
        lateral /= norm
        up = np.cross(tangent, lateral); up /= np.linalg.norm(up)
        basis = np.column_stack((tangent, lateral, up))
        if np.linalg.det(basis) <= 0.0:
            raise TemporalError("track frame is not right-handed")
        return TrackFrame(center, tangent.copy(), lateral, up)


def _normalize_quaternion(value: np.ndarray) -> np.ndarray:
    if value.shape != (4,) or not np.isfinite(value).all() or np.linalg.norm(value) <= 1e-12:
        raise TemporalError("trajectory contains an invalid quaternion")
    return value / np.linalg.norm(value)


def quaternion_slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    first = _normalize_quaternion(np.asarray(first, dtype=np.float64))
    second = _normalize_quaternion(np.asarray(second, dtype=np.float64))
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second; dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _normalize_quaternion(first + fraction * (second - first))
    theta = np.arccos(dot)
    return (np.sin((1-fraction)*theta)*first + np.sin(fraction*theta)*second) / np.sin(theta)


def quaternion_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quaternion(np.asarray(q, dtype=np.float64))
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def matrix_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64); trace = float(np.trace(matrix))
    if trace > 0:
        s = np.sqrt(trace+1.0)*2; q = ((matrix[2,1]-matrix[1,2])/s,(matrix[0,2]-matrix[2,0])/s,(matrix[1,0]-matrix[0,1])/s,.25*s)
    else:
        i = int(np.argmax(np.diag(matrix)))
        if i == 0:
            s=np.sqrt(1+matrix[0,0]-matrix[1,1]-matrix[2,2])*2; q=(.25*s,(matrix[0,1]+matrix[1,0])/s,(matrix[0,2]+matrix[2,0])/s,(matrix[2,1]-matrix[1,2])/s)
        elif i == 1:
            s=np.sqrt(1+matrix[1,1]-matrix[0,0]-matrix[2,2])*2; q=((matrix[0,1]+matrix[1,0])/s,.25*s,(matrix[1,2]+matrix[2,1])/s,(matrix[0,2]-matrix[2,0])/s)
        else:
            s=np.sqrt(1+matrix[2,2]-matrix[0,0]-matrix[1,1])*2; q=((matrix[0,2]+matrix[2,0])/s,(matrix[1,2]+matrix[2,1])/s,.25*s,(matrix[1,0]-matrix[0,1])/s)
    normalized = _normalize_quaternion(np.asarray(q)); return tuple(float(v) for v in normalized)


class CsvTrajectoryPoseProvider:
    def __init__(self, config: EgoMotionConfig) -> None:
        timestamps=[]; translations=[]; quaternions=[]
        try:
            with Path(config.path).open("r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                required={"timestamp_ns","x_m","y_m","z_m","qx","qy","qz","qw"}
                if reader.fieldnames is None or not required.issubset(reader.fieldnames):
                    raise TemporalError(f"trajectory CSV must contain columns {sorted(required)}")
                for row in reader:
                    timestamps.append(int(row["timestamp_ns"]))
                    translations.append([float(row[name]) for name in ("x_m","y_m","z_m")])
                    quaternions.append(_normalize_quaternion(np.array([float(row[name]) for name in ("qx","qy","qz","qw")])))
        except TemporalError: raise
        except Exception as exc: raise TemporalError(f"failed to read trajectory CSV {config.path}: {exc}") from exc
        self.timestamps=np.asarray(timestamps,dtype=np.int64); self.translations=np.asarray(translations); self.quaternions=np.asarray(quaternions)
        if len(self.timestamps)<2 or np.any(np.diff(self.timestamps)<=0):
            raise TemporalError("trajectory timestamps must be strictly increasing and contain at least two rows")
        if not np.isfinite(self.translations).all(): raise TemporalError("trajectory translations must be finite")
        self.max_gap_ns=int(config.max_interpolation_gap_ms*1_000_000)

    def pose_at(self, timestamp_ns: int) -> np.ndarray:
        if timestamp_ns < self.timestamps[0] or timestamp_ns > self.timestamps[-1]:
            raise TemporalError(f"trajectory does not permit extrapolation at {timestamp_ns}")
        right=int(np.searchsorted(self.timestamps,timestamp_ns,side="left"))
        if right < len(self.timestamps) and self.timestamps[right] == timestamp_ns:
            left=right; fraction=0.0
        else:
            left=right-1
            gap=int(self.timestamps[right]-self.timestamps[left])
            if gap > self.max_gap_ns: raise TemporalError(f"trajectory interpolation gap {gap} ns exceeds {self.max_gap_ns} ns")
            fraction=(timestamp_ns-self.timestamps[left])/gap
        translation=self.translations[left] if left==right else self.translations[left]+fraction*(self.translations[right]-self.translations[left])
        quaternion=self.quaternions[left] if left==right else quaternion_slerp(self.quaternions[left],self.quaternions[right],fraction)
        transform=np.eye(4); transform[:3,:3]=quaternion_matrix(quaternion); transform[:3,3]=translation
        return transform


@dataclass(frozen=True)
class FrameObjectState:
    config: ScenarioObject
    active: bool
    built: BuiltGeometry | None
    transform_reference: np.ndarray | None
    transform_lidar: np.ndarray | None
    track_state: dict[str, object] | None


class TemporalScene:
    """Resolve one consistent pose per object and PointCloud2 timestamp."""
    def __init__(self, scenario: Scenario) -> None:
        self.scenario=scenario
        self.track=PolylineTrack(scenario.track) if scenario.track else None
        self.provider=CsvTrajectoryPoseProvider(scenario.ego_motion) if scenario.ego_motion else None
        self.activation_timestamp: dict[str,int]={}
        self.templates={obj.object_id:build_geometry(obj,np.eye(4)) for obj in scenario.objects}

    def states_at(self, frame_index: int, timestamp_ns: int) -> tuple[FrameObjectState,...]:
        states=[]
        for obj in self.scenario.objects:
            temporal=obj.temporal
            active=temporal is None or temporal.start_index <= frame_index <= temporal.end_index
            if not active:
                states.append(FrameObjectState(obj,False,None,None,None,None)); continue
            self.activation_timestamp.setdefault(obj.object_id,timestamp_ns)
            dt=(timestamp_ns-self.activation_timestamp[obj.object_id])*1e-9
            placement=obj.placement
            if isinstance(placement,LidarPlacement):
                pose=PoseConfig("lidar",placement.xyz_m,placement.rpy_deg)
                lidar_transform=pose_transform(pose); reference_transform=None; track_state=None
            elif isinstance(placement,TrackPlacement):
                if self.track is None or self.provider is None: raise TemporalError("track-relative placement requires track and ego pose provider")
                motion=temporal.motion
                s=placement.longitudinal_m+motion.longitudinal_velocity_mps*dt
                lateral=placement.lateral_m+motion.lateral_velocity_mps*dt
                height=placement.height_m+motion.vertical_velocity_mps*dt
                rpy=(placement.rpy_track_deg[0],placement.rpy_track_deg[1],placement.rpy_track_deg[2]+motion.yaw_rate_deg_s*dt)
                frame=self.track.frame_at(s)
                base=np.column_stack((frame.tangent,frame.lateral,frame.up))
                local_rotation=pose_transform(PoseConfig("track",(0,0,0),rpy))[:3,:3]
                reference_transform=np.eye(4); reference_transform[:3,:3]=base@local_rotation
                reference_transform[:3,3]=frame.center+lateral*frame.lateral+height*frame.up
                lidar_transform=np.linalg.inv(self.provider.pose_at(timestamp_ns))@reference_transform
                track_state={"longitudinal_m":float(s),"lateral_m":float(lateral),"height_m":float(height),"rpy_track_deg":[float(v) for v in rpy]}
            else:
                raise TemporalError(f"{obj.object_id}: missing placement")
            states.append(FrameObjectState(obj,True,transform_built_geometry(self.templates[obj.object_id],lidar_transform),reference_transform,lidar_transform,track_state))
        return tuple(states)
