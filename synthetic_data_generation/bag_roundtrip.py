"""Identity roundtrip for ROS 2 bags.

Every message is deliberately deserialized, passed through ``process_message``,
serialized again, and written with its original topic and bag timestamp.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


class RoundtripError(RuntimeError):
    """Raised when a bag cannot be roundtripped safely."""


@dataclass(frozen=True)
class RoundtripResult:
    """Summary returned after a successful roundtrip."""

    input_path: Path
    output_path: Path
    storage_id: str
    message_count: int
    input_message_count: int
    output_message_count: int
    topic_counts: Mapping[str, int]
    elapsed_seconds: float


def process_message(
    topic_name: str,
    message: Any,
    timestamp: int,
    context: Optional[Any] = None,
) -> Any:
    """Process one message through the configured extension point.

    With no context this remains the original identity operation. A processing
    context may selectively modify a message while all other messages continue
    through the unchanged roundtrip path.
    """

    if context is not None:
        handler = getattr(context, "process_message", None)
        if handler is not None:
            return handler(topic_name, message, timestamp)
    del topic_name, timestamp
    return message


def _resolved(path: Path) -> Path:
    return path.expanduser().resolve(strict=False)


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def validate_paths(input_path: Path, output_path: Path) -> tuple[Path, Path]:
    """Resolve and validate input/output paths before any output is created."""

    resolved_input = _resolved(input_path)
    resolved_output = _resolved(output_path)

    if not resolved_input.exists():
        raise RoundtripError(f"Input bag does not exist: {resolved_input}")
    if resolved_input == resolved_output:
        raise RoundtripError("Input and output bag paths must be different.")
    if _is_relative_to(resolved_output, resolved_input):
        raise RoundtripError("Output bag must not be created inside the input bag.")
    if _is_relative_to(resolved_input, resolved_output):
        raise RoundtripError("Output path must not contain the input bag.")

    return resolved_input, resolved_output


def prepare_output_path(output_path: Path) -> None:
    """Require a new bag path without deleting any existing user data."""

    if output_path.exists():
        raise FileExistsError(
            f"Output path already exists: {output_path}. "
            "Choose a new, non-existing directory for the output bag."
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)


def detect_storage_id(input_path: Path) -> str:
    """Read the storage backend recorded in rosbag2 metadata."""

    try:
        metadata = rosbag2_py.Info().read_metadata(str(input_path), "")
    except Exception as exc:  # pybind exceptions differ between ROS releases
        raise RoundtripError(
            f"Failed to read rosbag2 metadata from {input_path}: {exc}"
        ) from exc

    storage_id = metadata.storage_identifier
    if not storage_id:
        raise RoundtripError(f"Bag metadata has no storage identifier: {input_path}")
    return storage_id


def open_reader(input_path: Path, storage_id: str) -> Any:
    """Open a sequential rosbag2 reader without serialization conversion."""

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def get_topic_metadata(reader: Any) -> list[Any]:
    """Return and validate the topics exposed by a reader."""

    topics = list(reader.get_all_topics_and_types())
    if not topics:
        raise RoundtripError("Input bag contains no topics.")

    names = [topic.name for topic in topics]
    duplicates = sorted(name for name, count in Counter(names).items() if count > 1)
    if duplicates:
        raise RoundtripError(f"Input bag has duplicate topic metadata: {duplicates}")
    return topics


def load_message_types(topic_metadata: Iterable[Any]) -> Dict[str, Any]:
    """Load every ROS message class before creating an output bag."""

    message_types: Dict[str, Any] = {}
    for topic in topic_metadata:
        if topic.serialization_format not in ("", "cdr"):
            raise RoundtripError(
                f"Topic {topic.name!r} uses unsupported serialization format "
                f"{topic.serialization_format!r}; this tool requires CDR."
            )
        try:
            message_types[topic.name] = get_message(topic.type)
        except Exception as exc:
            raise RoundtripError(
                f"Cannot load ROS message type {topic.type!r} for topic "
                f"{topic.name!r}. Ensure its package is installed and the ROS "
                f"environment is sourced. Original error: {exc}"
            ) from exc
    return message_types


def open_writer(
    output_path: Path,
    storage_id: str,
    topic_metadata: Iterable[Any],
) -> Any:
    """Open a writer with the input backend and recreate every input topic."""

    get_writers = getattr(rosbag2_py, "get_registered_writers", None)
    if get_writers is not None:
        registered = set(get_writers())
        if storage_id not in registered:
            raise RoundtripError(
                f"Storage backend {storage_id!r} is not available for writing. "
                f"Registered writers: {sorted(registered)}"
            )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in topic_metadata:
        writer.create_topic(topic)
    return writer


def roundtrip_bag(
    input_path: Path | str,
    output_path: Path | str,
    *,
    progress_every: int = 100,
    context: Optional[Any] = None,
) -> RoundtripResult:
    """Roundtrip all messages through ROS deserialization and serialization."""

    input_path, output_path = validate_paths(Path(input_path), Path(output_path))
    prepare_output_path(output_path)
    storage_id = detect_storage_id(input_path)

    print(f"Input bag:  {input_path}")
    print(f"Output bag: {output_path}")
    print(f"Storage:    {storage_id}")

    reader = open_reader(input_path, storage_id)
    topics = get_topic_metadata(reader)
    message_types = load_message_types(topics)
    if context is not None:
        validate_topics = getattr(context, "validate_topics", None)
        if validate_topics is not None:
            validate_topics(topics)
        prepare_outputs = getattr(context, "prepare_outputs", None)
        if prepare_outputs is not None:
            prepare_outputs(output_path)

    additional_topics: list[Any] = []
    if context is not None:
        additional = getattr(context, "additional_topic_metadata", None)
        if additional is not None:
            serialization_format = next(
                (topic.serialization_format for topic in topics if topic.serialization_format),
                "cdr",
            )
            additional_topics = list(additional(serialization_format))
    existing_names = {topic.name for topic in topics}
    duplicate_additions = [topic.name for topic in additional_topics if topic.name in existing_names]
    if duplicate_additions:
        raise RoundtripError(f"Additional output topics conflict with input topics: {duplicate_additions}")

    print("Topics:")
    for topic in topics:
        print(f"  {topic.name}: {topic.type} [{topic.serialization_format}]")
    for topic in additional_topics:
        print(f"  {topic.name}: {topic.type} [{topic.serialization_format}] (generated)")

    writer = None
    counts: Counter[str] = Counter()
    input_total = 0
    output_total = 0
    started = time.perf_counter()

    try:
        writer = open_writer(output_path, storage_id, [*topics, *additional_topics])
        while reader.has_next():
            topic_name, serialized_data, timestamp = reader.read_next()
            message_type = message_types.get(topic_name)
            if message_type is None:
                raise RoundtripError(
                    f"Message references unknown topic {topic_name!r}."
                )

            try:
                message = deserialize_message(serialized_data, message_type)
                processed = process_message(
                    topic_name,
                    message,
                    timestamp,
                    context,
                )
                output_data = serialize_message(processed)
            except Exception as exc:
                raise RoundtripError(
                    f"Failed to roundtrip topic {topic_name!r} at timestamp "
                    f"{timestamp}: {exc}"
                ) from exc

            writer.write(topic_name, output_data, timestamp)
            counts[topic_name] += 1
            input_total += 1
            output_total += 1

            if context is not None:
                pop_extra = getattr(context, "pop_extra_messages", None)
                if pop_extra is not None:
                    for extra_topic, extra_message, extra_timestamp in pop_extra():
                        try:
                            extra_data = serialize_message(extra_message)
                        except Exception as exc:
                            raise RoundtripError(
                                f"Failed to serialize generated message on {extra_topic!r}: {exc}"
                            ) from exc
                        writer.write(extra_topic, extra_data, extra_timestamp)
                        counts[extra_topic] += 1
                        output_total += 1

            if progress_every > 0 and input_total % progress_every == 0:
                print(f"Processed {input_total} input messages...")
        if context is not None:
            finalize = getattr(context, "finalize", None)
            if finalize is not None:
                finalize()
    except Exception:
        if context is not None:
            abort = getattr(context, "abort", None)
            if abort is not None:
                abort()
        if writer is not None:
            print(
                f"warning: output bag may be incomplete: {output_path}",
                file=sys.stderr,
            )
        raise
    finally:
        if writer is not None:
            close = getattr(writer, "close", None)
            if close is not None:
                close()

    elapsed = time.perf_counter() - started
    print("Message counts:")
    for topic in [*topics, *additional_topics]:
        print(f"  {topic.name}: {counts[topic.name]}")
    print(f"Input total: {input_total}")
    print(f"Output total: {output_total}")
    print(f"Elapsed: {elapsed:.3f} s")
    if context is not None:
        print_summary = getattr(context, "print_summary", None)
        if print_summary is not None:
            print_summary(input_total, output_total, output_path)
    print("Roundtrip completed successfully.")

    return RoundtripResult(
        input_path=input_path,
        output_path=output_path,
        storage_id=storage_id,
        message_count=output_total,
        input_message_count=input_total,
        output_message_count=output_total,
        topic_counts=dict(counts),
        elapsed_seconds=elapsed,
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Deserialize, identity-process, serialize, and rewrite every "
            "message in a ROS 2 bag."
        )
    )
    parser.add_argument("--input", required=True, type=Path, help="Input bag URI")
    parser.add_argument("--output", required=True, type=Path, help="Output bag URI")
    parser.add_argument(
        "--scenario",
        type=Path,
        help="Schema-v1/v2 YAML scenario (cannot be combined with legacy cube options)",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=100,
        help="Print progress every N messages; use 0 to disable",
    )
    parser.add_argument(
        "--pointcloud-topic",
        help="PointCloud2 topic selected for optional cube injection",
    )
    parser.add_argument(
        "--target-frame-index",
        type=int,
        help="Zero-based frame index counted only on --pointcloud-topic",
    )
    parser.add_argument(
        "--inject-cube",
        action="store_true",
        help="Inject one axis-aligned Open3D cube into the selected frame",
    )
    for axis in ("x", "y", "z"):
        parser.add_argument(
            f"--cube-center-{axis}",
            type=float,
            help=f"Cube center {axis.upper()} in the LiDAR frame, metres",
        )
        parser.add_argument(
            f"--cube-size-{axis}",
            type=float,
            help=f"Cube size along {axis.upper()}, metres",
        )
    return parser


def build_processing_context(args: argparse.Namespace) -> Optional[Any]:
    """Build and validate the optional cube-injection context."""

    cube_values = (
        args.cube_center_x,
        args.cube_center_y,
        args.cube_center_z,
        args.cube_size_x,
        args.cube_size_y,
        args.cube_size_z,
    )
    cube_options_supplied = any(value is not None for value in cube_values)
    selector_supplied = (
        args.pointcloud_topic is not None or args.target_frame_index is not None
    )

    if args.scenario is not None:
        if args.inject_cube or cube_options_supplied or selector_supplied:
            raise RoundtripError(
                "--scenario cannot be combined with legacy --inject-cube, "
                "--pointcloud-topic, --target-frame-index, or --cube-* options."
            )
        from synthetic_data_generation.processor import ProcessingContext
        from synthetic_data_generation.scenario import load_scenario

        return ProcessingContext.from_scenario(load_scenario(args.scenario))

    if not args.inject_cube:
        if cube_options_supplied or selector_supplied:
            raise RoundtripError(
                "Cube/topic options require the --inject-cube flag."
            )
        return None

    if not args.pointcloud_topic:
        raise RoundtripError("--pointcloud-topic is required with --inject-cube.")
    if args.target_frame_index is None or args.target_frame_index < 0:
        raise RoundtripError(
            "--target-frame-index must be a non-negative integer."
        )
    if any(value is None for value in cube_values):
        raise RoundtripError(
            "All --cube-center-* and --cube-size-* values are required "
            "with --inject-cube."
        )

    from synthetic_data_generation.cube_injector import CubeConfig
    from synthetic_data_generation.processor import ProcessingContext

    cube = CubeConfig(
        center=(args.cube_center_x, args.cube_center_y, args.cube_center_z),
        size=(args.cube_size_x, args.cube_size_y, args.cube_size_z),
    )
    return ProcessingContext(
        pointcloud_topic=args.pointcloud_topic,
        target_frame_index=args.target_frame_index,
        cube=cube,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        context = build_processing_context(args)
        roundtrip_bag(
            args.input,
            args.output,
            progress_every=args.progress_every,
            context=context,
        )
    except (RoundtripError, FileExistsError, ValueError, ImportError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
