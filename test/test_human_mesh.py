from pathlib import Path

import numpy as np
import pytest

from synthetic_data_generation.geometry import GeometryError, build_geometries
from synthetic_data_generation.scenario import parse_scenario
from test_geometry_v2 import obj, v2_document


def test_human_obj_units_scale_and_base_center():
    pytest.importorskip("trimesh", reason="Trimesh is not installed")
    mesh = Path(__file__).parents[1] / "meshes" / "humans" / "synthetic_person.obj"
    geometry = {"type": "human_mesh", "path": str(mesh), "units": "centimeters",
                "scale": [2,1,1], "origin": "base_center",
                "mesh_resource_uri": "package://synthetic_data_generation/meshes/humans/synthetic_person.obj"}
    scenario = parse_scenario(v2_document([obj("human", geometry, xyz=(10,2,3))]))
    built = build_geometries(scenario.objects)[0]
    assert built.bounds_min[2] == pytest.approx(3.0)
    assert (built.bounds_min[0] + built.bounds_max[0]) / 2 == pytest.approx(10.0)
    assert (built.bounds_min[1] + built.bounds_max[1]) / 2 == pytest.approx(2.0)
    assert built.mesh_sha256 and len(built.mesh_sha256) == 64
    assert np.isfinite(built.vertices_lidar).all()


def test_human_point_cloud_without_faces_is_rejected(tmp_path: Path):
    pytest.importorskip("trimesh", reason="Trimesh is not installed")
    mesh = tmp_path / "points.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\n", encoding="utf-8")
    geometry = {"type": "human_mesh", "path": str(mesh), "units": "meters",
                "scale": [1,1,1], "origin": "mesh_origin"}
    scenario = parse_scenario(v2_document([obj("human", geometry)]))
    with pytest.raises(GeometryError, match="triangular"):
        build_geometries(scenario.objects)
