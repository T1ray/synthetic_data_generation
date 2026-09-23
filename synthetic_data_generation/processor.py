"""Frame selection, synthetic geometry, annotations, and supplemental markers."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
import time
from typing import Any, Iterable, Optional

import numpy as np
from sensor_msgs.msg import PointCloud2

from synthetic_data_generation.bag_roundtrip import RoundtripError
from synthetic_data_generation.cube_injector import CubeConfig, InjectionStats, inject_cube
from synthetic_data_generation.geometry import BuiltGeometry, build_geometries
from synthetic_data_generation.ground_truth import AnnotationWriter, annotation_path_for
from synthetic_data_generation.object_injector import FrameInjectionResult, inject_objects
from synthetic_data_generation.scenario import Scenario
from synthetic_data_generation.visualization import build_clear_marker_array, build_marker_array
from synthetic_data_generation.temporal import TemporalScene, matrix_quaternion


@dataclass
class ProcessingContext:
    pointcloud_topic: str
    cube: Optional[CubeConfig] = None
    target_frame_index: Optional[int] = None
    start_index: Optional[int] = None
    end_index: Optional[int] = None
    scenario: Optional[Scenario] = None
    frames_seen: int = 0
    injection_performed: bool = False
    injection_stats: Optional[InjectionStats] = None
    last_frame_result: Optional[FrameInjectionResult] = None
    processed_indices: list[int] = field(default_factory=list)
    valid_ray_count: int = 0
    ray_intersection_count: int = 0
    modified_slot_count: int = 0
    zero_visible_frames: int = 0
    marker_array_count: int = 0
    annotations_path: Optional[Path] = None
    geometries: tuple[BuiltGeometry, ...] = field(default_factory=tuple)
    object_modified_counts: Counter[str] = field(default_factory=Counter)
    scene_build_seconds: float = 0.0
    injection_seconds: float = 0.0
    marker_seconds: float = 0.0
    _annotations: Optional[AnnotationWriter] = field(default=None, init=False, repr=False)
    _extra_messages: list[tuple[str, Any, int]] = field(default_factory=list, init=False, repr=False)
    _temporal_scene: Optional[TemporalScene] = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.target_frame_index is not None:
            if self.start_index is not None or self.end_index is not None:
                raise ValueError("Use either target_frame_index or a frame range, not both")
            self.start_index = self.target_frame_index
            self.end_index = self.target_frame_index
        if self.start_index is None or self.end_index is None:
            raise ValueError("A target frame or inclusive frame range is required")
        if self.start_index < 0 or self.end_index < 0 or self.start_index > self.end_index:
            raise ValueError("Frame range must satisfy 0 <= start_index <= end_index")
        if self.scenario is None and self.cube is None:
            raise ValueError("Legacy processing requires CubeConfig")

    @classmethod
    def from_scenario(cls, scenario: Scenario) -> "ProcessingContext":
        legacy_cube = None
        if scenario.schema_version == 1:
            legacy_cube = CubeConfig(center=scenario.object.xyz_m, size=scenario.object.dimensions_m)
        return cls(pointcloud_topic=scenario.pointcloud_topic, cube=legacy_cube,
                   start_index=scenario.start_index, end_index=scenario.end_index,
                   scenario=scenario)

    @property
    def visualization_enabled(self) -> bool:
        return bool(self.scenario and self.scenario.visualization.enabled)

    def validate_topics(self, topic_metadata: Iterable[Any]) -> None:
        topics = {topic.name: topic.type for topic in topic_metadata}
        if self.pointcloud_topic not in topics:
            raise RoundtripError(f"Selected PointCloud2 topic was not found: {self.pointcloud_topic}")
        actual_type = topics[self.pointcloud_topic]
        if actual_type != "sensor_msgs/msg/PointCloud2":
            raise RoundtripError(f"Selected topic {self.pointcloud_topic!r} has type {actual_type!r}, expected 'sensor_msgs/msg/PointCloud2'.")
        if self.visualization_enabled:
            marker_topic = self.scenario.visualization.marker_topic
            if marker_topic in topics:
                raise RoundtripError(f"Marker topic {marker_topic!r} already exists in the input bag with type {topics[marker_topic]!r}")
        if self.scenario:
            print(f"Scenario ID: {self.scenario.scenario_id}")
            print(f"Objects: {len(self.scenario.objects)}")
            for obj in self.scenario.objects:
                if obj.pose is not None:
                    pose_text=f"xyz={obj.xyz_m}, rpy={obj.rpy_deg}"
                else:
                    pose_text=f"placement={obj.placement.type}, active=[{obj.temporal.start_index}, {obj.temporal.end_index}]"
                print(f"  {obj.object_id}: class={obj.class_name}, geometry={obj.geometry_type}, {pose_text}")
        print(f"PointCloud2 topic: {self.pointcloud_topic}")
        print(f"Requested inclusive frame range: [{self.start_index}, {self.end_index}]")
        if self.visualization_enabled:
            print(f"Marker topic: {self.scenario.visualization.marker_topic}")

    def prepare_outputs(self, output_bag: Path) -> None:
        if self.scenario is None:
            return
        if self.visualization_enabled:
            try:
                from visualization_msgs.msg import MarkerArray  # noqa: F401
            except ImportError as exc:
                raise ImportError("visualization_msgs is required when visualization.enabled is true") from exc
        self.annotations_path = annotation_path_for(output_bag)
        annotations = AnnotationWriter(self.annotations_path)
        if self.scenario.schema_version == 2:
            self.geometries = build_geometries(self.scenario.objects)
        elif self.scenario.schema_version == 3:
            self._temporal_scene = TemporalScene(self.scenario)
        self._annotations = annotations
        self._annotations.open()
        if self.scenario.schema_version == 2:
            for item in self.geometries:
                print(f"Built {item.object_id}: vertices={len(item.vertices_lidar)}, triangles={len(item.triangles)}")

    def additional_topic_metadata(self, serialization_format: str) -> list[Any]:
        if not self.visualization_enabled:
            return []
        try:
            import rosbag2_py
            from visualization_msgs.msg import MarkerArray  # noqa: F401
        except ImportError as exc:
            raise ImportError("visualization_msgs is required when visualization.enabled is true") from exc
        return [rosbag2_py.TopicMetadata(
            name=self.scenario.visualization.marker_topic,
            type="visualization_msgs/msg/MarkerArray",
            serialization_format=serialization_format or "cdr",
        )]

    def pop_extra_messages(self) -> list[tuple[str, Any, int]]:
        result = self._extra_messages
        self._extra_messages = []
        return result

    def _frame_rng(self, frame_index: int) -> np.random.Generator:
        seed = self.scenario.seed if self.scenario is not None else 0
        return np.random.default_rng(np.random.SeedSequence([seed, frame_index]))

    @staticmethod
    def _header_stamp_ns(message: PointCloud2) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _legacy_annotation(self, message: PointCloud2, topic_name: str, timestamp: int,
                           frame_index: int, stats: InjectionStats) -> None:
        obj = self.scenario.object
        self._annotations.write({
            "schema_version": 1, "scenario_id": self.scenario.scenario_id, "seed": self.scenario.seed,
            "topic": topic_name, "frame_index": frame_index, "bag_timestamp_ns": int(timestamp),
            "header_stamp_ns": self._header_stamp_ns(message), "input_point_count": stats.total_points,
            "valid_ray_count": stats.valid_rays,
            "objects": [{"object_id": obj.object_id, "class_name": obj.class_name,
                "geometry_type": obj.geometry_type, "coordinate_system": obj.coordinate_system,
                "pose_lidar": {"xyz_m": list(obj.xyz_m), "rpy_deg": list(obj.rpy_deg)},
                "dimensions_m": list(obj.dimensions_m), "ray_intersection_count": stats.cube_hits,
                "visible_point_count": stats.replaced_points, "modified_slot_count": stats.replaced_points}],
        })

    def _v2_annotation(self, message: PointCloud2, topic_name: str, timestamp: int,
                       frame_index: int, result: FrameInjectionResult) -> None:
        stats = {item.object_id: item for item in result.object_stats}
        objects = []
        for config, built in zip(self.scenario.objects, self.geometries):
            item_stats = stats[config.object_id]
            record = {
                "object_id": config.object_id, "class_name": config.class_name,
                "geometry_type": config.geometry_type, "coordinate_system": config.coordinate_system,
                "pose_lidar": {"xyz_m": list(config.xyz_m), "rpy_deg": list(config.rpy_deg)},
                "bounds_lidar": {"min_xyz_m": list(built.bounds_min), "max_xyz_m": list(built.bounds_max)},
                "ray_intersection_count": item_stats.ray_intersection_count,
                "visible_point_count": item_stats.visible_point_count,
                "modified_slot_count": item_stats.modified_slot_count,
            }
            if built.mesh_path:
                record["mesh_path"] = built.mesh_path
                record["mesh_sha256"] = built.mesh_sha256
            objects.append(record)
        self._annotations.write({
            "schema_version": 2, "scenario_id": self.scenario.scenario_id, "seed": self.scenario.seed,
            "topic": topic_name, "frame_index": frame_index, "bag_timestamp_ns": int(timestamp),
            "header_stamp_ns": self._header_stamp_ns(message), "input_point_count": result.total_points,
            "valid_ray_count": result.valid_ray_count, "modified_slot_count": result.modified_slot_count,
            "objects": objects,
        })

    @staticmethod
    def _pose_record(transform: Optional[np.ndarray]) -> Optional[dict[str, Any]]:
        if transform is None:
            return None
        return {"xyz_m": [float(v) for v in transform[:3,3]],
                "quaternion_xyzw": list(matrix_quaternion(transform[:3,:3]))}

    def _v3_annotation(self, message: PointCloud2, topic_name: str, timestamp: int,
                       frame_index: int, reference_timestamp: int, states: tuple[Any,...],
                       result: FrameInjectionResult) -> None:
        stats={item.object_id:item for item in result.object_stats}; built={item.object_id:item for item in self.geometries}
        objects=[]
        for state in states:
            obj=state.config; item_stats=stats.get(obj.object_id)
            record={"object_id":obj.object_id,"class_name":obj.class_name,"geometry_type":obj.geometry_type,
                    "active":state.active,"placement_type":obj.placement.type,
                    "pose_track":state.track_state,"pose_reference":self._pose_record(state.transform_reference),
                    "pose_lidar":self._pose_record(state.transform_lidar),
                    "ideal_visible_hit_count":item_stats.ideal_visible_hit_count if item_stats else 0,
                    "returned_after_dropout_count":item_stats.returned_after_dropout_count if item_stats else 0,
                    "dropout_count":item_stats.dropout_count if item_stats else 0,
                    "synthetic_returns_from_zero_slots":item_stats.synthetic_returns_from_zero_slots if item_stats else 0,
                    "ray_intersection_count":item_stats.ray_intersection_count if item_stats else 0,
                    "visible_point_count":item_stats.visible_point_count if item_stats else 0,
                    "modified_slot_count":item_stats.modified_slot_count if item_stats else 0}
            if state.active:
                geometry=built[obj.object_id]
                record["bounds_lidar"]={"min_xyz_m":list(geometry.bounds_min),"max_xyz_m":list(geometry.bounds_max)}
                if geometry.mesh_path: record.update(mesh_path=geometry.mesh_path,mesh_sha256=geometry.mesh_sha256)
            objects.append(record)
        zero=result.zero_slot_stats; effects=result.sensor_effect_stats
        self._annotations.write({"schema_version":3,"sensor_model_version":1,
            "scenario_id":self.scenario.scenario_id,"seed":self.scenario.seed,"topic":topic_name,
            "frame_index":frame_index,"bag_timestamp_ns":int(timestamp),"header_stamp_ns":self._header_stamp_ns(message),
            "reference_timestamp_ns":int(reference_timestamp),"input_point_count":result.total_points,
            "valid_ray_count":result.valid_ray_count,"modified_slot_count":result.modified_slot_count,
            "zero_slot_count":zero.zero_slot_count,"recoverable_zero_slot_count":zero.recoverable_zero_slot_count,
            "recovered_direction_count":zero.recovered_direction_count,
            "synthetic_returns_from_zero_slots":sum(item.synthetic_returns_from_zero_slots for item in result.object_stats),
            "zero_slot_rejection_reasons":zero.rejection_reasons,
            "direction_quality":{"median_angular_error_deg":zero.median_angular_error_deg,
                "p95_angular_error_deg":zero.p95_angular_error_deg,"max_angular_error_deg":zero.max_angular_error_deg},
            "sensor_effects":{"range_noise_enabled":self.scenario.sensor_effects.range_noise.enabled,
                "dropout_enabled":self.scenario.sensor_effects.dropout.enabled,
                "intensity_enabled":self.scenario.sensor_effects.intensity.enabled},
            "range_noise_applied_count":effects.range_noise_applied_count,
            "range_noise_resample_count":effects.range_noise_resample_count,
            "range_noise_clamp_count":effects.range_noise_clamp_count,"dropout_count":effects.dropout_count,
            "intensity_generated_count":effects.intensity_generated_count,
            "intensity_fallback_counts":effects.intensity_fallback_counts,
            "normal_fallback_count":effects.normal_fallback_count,"objects":objects})

    def process_message(self, topic_name: str, message: Any, timestamp: int) -> Any:
        if topic_name != self.pointcloud_topic:
            return message
        if not isinstance(message, PointCloud2):
            raise RoundtripError(f"Selected topic {topic_name!r} did not deserialize as PointCloud2.")
        current_index = self.frames_seen
        self.frames_seen += 1
        if (current_index == self.end_index + 1 and self.visualization_enabled
                and self.scenario.visualization.marker_mode == "frame"):
            self._extra_messages.append((self.scenario.visualization.marker_topic,
                                         build_clear_marker_array(message), int(timestamp)))
            self.marker_array_count += 1
        if current_index < self.start_index or current_index > self.end_index:
            return message
        self._frame_rng(current_index)

        if self.scenario is None or self.scenario.schema_version == 1:
            stats = inject_cube(message, self.cube)
            self.injection_stats = stats
            valid_rays, hits, modified = stats.valid_rays, stats.cube_hits, stats.replaced_points
            if self.scenario is not None:
                self._legacy_annotation(message, topic_name, timestamp, current_index, stats)
                self.object_modified_counts[self.scenario.object.object_id] += modified
        else:
            reference_timestamp=self._header_stamp_ns(message) or int(timestamp)
            states=None
            if self.scenario.schema_version == 3:
                stage_started=time.perf_counter()
                states=self._temporal_scene.states_at(current_index,reference_timestamp)
                self.geometries=tuple(state.built for state in states if state.active)
                materials={state.config.object_id:state.config.material for state in states if state.active}
                self.scene_build_seconds+=time.perf_counter()-stage_started
                stage_started=time.perf_counter()
                result=inject_objects(message,self.geometries,scenario_seed=self.scenario.seed,
                    frame_index=current_index,zero_slot_config=self.scenario.zero_slot_recovery,
                    sensor_effects=self.scenario.sensor_effects,materials=materials)
                self.injection_seconds+=time.perf_counter()-stage_started
            else:
                stage_started=time.perf_counter()
                result = inject_objects(message, self.geometries)
                self.injection_seconds+=time.perf_counter()-stage_started
            self.last_frame_result = result
            valid_rays, hits, modified = result.valid_ray_count, result.ray_intersection_count, result.modified_slot_count
            if self.scenario.schema_version == 3:
                self._v3_annotation(message,topic_name,timestamp,current_index,reference_timestamp,states,result)
            else:
                self._v2_annotation(message, topic_name, timestamp, current_index, result)
            for item in result.object_stats:
                self.object_modified_counts[item.object_id] += item.modified_slot_count
            if self.visualization_enabled:
                marker_started=time.perf_counter()
                marker_array = build_marker_array(message, self.geometries, result, self.scenario.visualization)
                self.marker_seconds+=time.perf_counter()-marker_started
                self._extra_messages.append((self.scenario.visualization.marker_topic, marker_array, int(timestamp)))
                self.marker_array_count += 1

        self.injection_performed = True
        self.processed_indices.append(current_index)
        self.valid_ray_count += valid_rays
        self.ray_intersection_count += hits
        self.modified_slot_count += modified
        if modified == 0:
            self.zero_visible_frames += 1
        return message

    def finalize(self) -> None:
        try:
            expected = list(range(self.start_index, self.end_index + 1))
            if self.processed_indices != expected:
                missing = [index for index in expected if index not in self.processed_indices]
                raise RoundtripError(f"Requested frame range [{self.start_index}, {self.end_index}] is not fully available on {self.pointcloud_topic}; observed {self.frames_seen} frames, missing indices: {missing}.")
        finally:
            if self._annotations is not None:
                self._annotations.close()

    def abort(self) -> None:
        if self._annotations is not None:
            self._annotations.close()

    def print_summary(self, input_messages: int, output_messages: int, output_bag: Path) -> None:
        print("Generation summary:")
        print(f"  input/output messages: {input_messages}/{output_messages}")
        print(f"  selected-topic frames: {self.frames_seen}")
        print(f"  requested range: [{self.start_index}, {self.end_index}]")
        print(f"  processed indices/count: {self.processed_indices}/{len(self.processed_indices)}")
        print(f"  MarkerArray messages: {self.marker_array_count}")
        print(f"  valid rays: {self.valid_ray_count}")
        print(f"  ray intersections: {self.ray_intersection_count}")
        print(f"  modified slots: {self.modified_slot_count}")
        for object_id, count in self.object_modified_counts.items():
            print(f"  object {object_id}: modified slots={count}")
        print(f"  zero-visible frames: {self.zero_visible_frames}")
        print(f"  pose/geometry update time: {self.scene_build_seconds:.6f} s")
        print(f"  ray/recovery/sensor time: {self.injection_seconds:.6f} s")
        print(f"  MarkerArray build time: {self.marker_seconds:.6f} s")
        print(f"  output bag: {output_bag}")
        if self.annotations_path is not None:
            print(f"  annotations JSONL: {self.annotations_path}")
