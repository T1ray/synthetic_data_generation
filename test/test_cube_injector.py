import copy
import importlib.util

import numpy as np
import pytest

from synthetic_data_generation.cube_injector import (
    CubeConfig,
    Open3DUnavailableError,
    inject_cube,
)
from synthetic_data_generation.pointcloud_codec import decode_cloud
from synthetic_data_generation.smoke_test import make_padded_point_cloud


def _require_open3d():
    return pytest.importorskip("open3d", reason="Open3D is not installed")


def test_cube_configuration_validation():
    with pytest.raises(ValueError, match="positive"):
        CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 0.0, 2.0))
    with pytest.raises(ValueError, match="finite"):
        CubeConfig(center=(np.nan, 0.0, 0.0), size=(2.0, 2.0, 2.0))


def test_missing_open3d_has_clear_error():
    if importlib.util.find_spec("open3d") is not None:
        pytest.skip("Open3D is installed")
    message = make_padded_point_cloud([(20.0, 0.0, 0.0)])
    original_data = bytes(message.data)
    with pytest.raises(Open3DUnavailableError, match="Open3D is required"):
        inject_cube(
            message,
            CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
        )
    assert bytes(message.data) == original_data


def test_cube_in_front_replaces_only_xyz():
    _require_open3d()
    message = make_padded_point_cloud([(20.0, 0.0, 0.0)])
    original = copy.deepcopy(message)
    original_decoded = decode_cloud(original)

    stats = inject_cube(
        message,
        CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    result = decode_cloud(message)

    assert stats.replaced_points == 1
    assert result.points["x"][0, 0] == pytest.approx(9.0, abs=1e-4)
    assert result.points["y"][0, 0] == pytest.approx(0.0)
    assert result.points["z"][0, 0] == pytest.approx(0.0)
    for field in ("intensity", "ring", "return_type", "timestamp"):
        assert np.array_equal(result.points[field], original_decoded.points[field])


def test_ray_missing_cube_is_byte_identical():
    _require_open3d()
    message = make_padded_point_cloud([(20.0, 5.0, 0.0)])
    original_data = bytes(message.data)
    stats = inject_cube(
        message,
        CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    assert stats.replaced_points == 0
    assert bytes(message.data) == original_data


def test_cube_behind_background_is_byte_identical():
    _require_open3d()
    message = make_padded_point_cloud([(5.0, 0.0, 0.0)])
    original_data = bytes(message.data)
    stats = inject_cube(
        message,
        CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    assert stats.cube_hits == 1
    assert stats.replaced_points == 0
    assert bytes(message.data) == original_data


def test_invalid_points_are_skipped_and_unchanged():
    _require_open3d()
    message = make_padded_point_cloud(
        [
            (0.0, 0.0, 0.0),
            (np.nan, 0.0, 0.0),
            (np.inf, 0.0, 0.0),
        ]
    )
    original_data = bytes(message.data)
    stats = inject_cube(
        message,
        CubeConfig(center=(10.0, 0.0, 0.0), size=(2.0, 2.0, 2.0)),
    )
    assert stats.valid_rays == 0
    assert stats.invalid_points == 3
    assert bytes(message.data) == original_data
