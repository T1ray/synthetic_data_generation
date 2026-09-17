import copy

import pytest
from sensor_msgs.msg import PointField

from synthetic_data_generation.pointcloud_codec import (
    PointCloudCodecError,
    decode_cloud,
    encode_cloud,
)
from synthetic_data_generation.smoke_test import make_padded_point_cloud


def test_identity_codec_preserves_bytes_metadata_and_padding():
    original = make_padded_point_cloud(
        [
            (1.0, 2.0, 3.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0),
            (-1.0, -2.0, -3.0),
            (-4.0, -5.0, -6.0),
            (-7.0, -8.0, -9.0),
        ],
        height=2,
        width=3,
        row_padding=11,
    )
    expected = copy.deepcopy(original)

    decoded = decode_cloud(original)
    encoded = encode_cloud(original, decoded)

    assert bytes(encoded.data) == bytes(expected.data)
    assert encoded.header == expected.header
    assert encoded.height == expected.height
    assert encoded.width == expected.width
    assert encoded.fields == expected.fields
    assert encoded.is_bigendian == expected.is_bigendian
    assert encoded.point_step == expected.point_step
    assert encoded.row_step == expected.row_step
    assert encoded.is_dense == expected.is_dense
    assert decoded.points.shape == (2, 3)
    assert decoded.points.strides == (expected.row_step, expected.point_step)


def test_big_endian_cloud_is_rejected():
    message = make_padded_point_cloud([(1.0, 0.0, 0.0)])
    message.is_bigendian = True
    with pytest.raises(PointCloudCodecError, match="Big-endian"):
        decode_cloud(message)


def test_missing_coordinate_field_is_rejected():
    message = make_padded_point_cloud([(1.0, 0.0, 0.0)])
    message.fields = [field for field in message.fields if field.name != "z"]
    with pytest.raises(PointCloudCodecError, match="missing required"):
        decode_cloud(message)


def test_integer_coordinate_field_is_rejected():
    message = make_padded_point_cloud([(1.0, 0.0, 0.0)])
    for field in message.fields:
        if field.name == "x":
            field.datatype = PointField.INT32
    with pytest.raises(PointCloudCodecError, match="FLOAT32 or FLOAT64"):
        decode_cloud(message)


def test_corrupt_data_size_is_rejected():
    message = make_padded_point_cloud([(1.0, 0.0, 0.0)])
    message.data = message.data[:-1]
    with pytest.raises(PointCloudCodecError, match="expected exactly"):
        decode_cloud(message)

