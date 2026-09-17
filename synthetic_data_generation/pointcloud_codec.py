"""Layout-preserving NumPy codec for ``sensor_msgs/msg/PointCloud2``."""

from __future__ import annotations

import array
from dataclasses import dataclass
from typing import Dict

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class PointCloudCodecError(ValueError):
    """Raised when a PointCloud2 layout cannot be processed safely."""


SUPPORTED_COORDINATE_TYPES = {PointField.FLOAT32, PointField.FLOAT64}


@dataclass
class DecodedPointCloud:
    """Writable structured point view backed by a complete message buffer."""

    buffer: bytearray
    points: np.ndarray
    dtype: np.dtype
    height: int
    width: int
    point_step: int
    row_step: int


def _field_map(message: PointCloud2) -> Dict[str, PointField]:
    fields = {field.name: field for field in message.fields}
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise PointCloudCodecError(
            f"PointCloud2 is missing required coordinate fields: {sorted(missing)}"
        )
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.count != 1 or field.datatype not in SUPPORTED_COORDINATE_TYPES:
            raise PointCloudCodecError(
                f"Coordinate field {name!r} must be a scalar FLOAT32 or FLOAT64; "
                f"datatype={field.datatype}, count={field.count}."
            )
    return fields


def _validate_layout(message: PointCloud2) -> np.dtype:
    if not isinstance(message, PointCloud2):
        raise PointCloudCodecError("decode_cloud expects sensor_msgs/msg/PointCloud2.")
    if message.is_bigendian:
        raise PointCloudCodecError(
            "Big-endian PointCloud2 layouts are not supported safely."
        )
    if message.point_step <= 0:
        raise PointCloudCodecError("PointCloud2.point_step must be positive.")
    if message.height < 0 or message.width < 0:
        raise PointCloudCodecError("PointCloud2 dimensions must be non-negative.")

    minimum_row_step = message.width * message.point_step
    if message.row_step < minimum_row_step:
        raise PointCloudCodecError(
            f"PointCloud2.row_step={message.row_step} is smaller than "
            f"width*point_step={minimum_row_step}."
        )
    expected_size = message.row_step * message.height
    if len(message.data) != expected_size:
        raise PointCloudCodecError(
            f"PointCloud2.data has {len(message.data)} bytes, expected exactly "
            f"row_step*height={expected_size}."
        )

    _field_map(message)
    try:
        dtype = point_cloud2.dtype_from_fields(
            message.fields,
            point_step=message.point_step,
        )
    except Exception as exc:
        raise PointCloudCodecError(f"Cannot construct PointCloud2 dtype: {exc}") from exc
    if dtype.itemsize != message.point_step:
        raise PointCloudCodecError(
            f"NumPy record size {dtype.itemsize} does not match point_step "
            f"{message.point_step}."
        )
    return dtype


def decode_cloud(message: PointCloud2) -> DecodedPointCloud:
    """Copy the full data buffer and expose a writable strided point view."""

    dtype = _validate_layout(message)
    buffer = bytearray(bytes(message.data))
    try:
        points = np.ndarray(
            shape=(message.height, message.width),
            dtype=dtype,
            buffer=buffer,
            strides=(message.row_step, message.point_step),
        )
    except Exception as exc:
        raise PointCloudCodecError(
            f"Cannot create a strided NumPy view for PointCloud2: {exc}"
        ) from exc
    if not points.flags.writeable:
        raise PointCloudCodecError("Decoded PointCloud2 view is unexpectedly read-only.")
    return DecodedPointCloud(
        buffer=buffer,
        points=points,
        dtype=dtype,
        height=message.height,
        width=message.width,
        point_step=message.point_step,
        row_step=message.row_step,
    )


def encode_cloud(
    message: PointCloud2,
    decoded: DecodedPointCloud,
) -> PointCloud2:
    """Write a decoded buffer back without changing PointCloud2 metadata."""

    dtype = _validate_layout(message)
    if decoded.height != message.height or decoded.width != message.width:
        raise PointCloudCodecError("Decoded point count/dimensions were changed.")
    if decoded.point_step != message.point_step or decoded.row_step != message.row_step:
        raise PointCloudCodecError("Decoded PointCloud2 strides were changed.")
    if decoded.dtype != dtype or decoded.points.dtype != dtype:
        raise PointCloudCodecError("Decoded NumPy dtype no longer matches PointCloud2 fields.")
    if decoded.points.shape != (message.height, message.width):
        raise PointCloudCodecError("Decoded NumPy shape no longer matches PointCloud2.")
    if decoded.points.strides != (message.row_step, message.point_step):
        raise PointCloudCodecError("Decoded NumPy strides no longer match PointCloud2.")

    expected_size = message.row_step * message.height
    if len(decoded.buffer) != expected_size:
        raise PointCloudCodecError(
            f"Encoded buffer has {len(decoded.buffer)} bytes; expected {expected_size}."
        )
    message.data = array.array("B", decoded.buffer)
    return message

