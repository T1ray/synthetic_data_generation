from glob import glob
from setuptools import find_packages, setup


package_name = "synthetic_data_generation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config/scenarios", glob("config/scenarios/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/meshes/humans", glob("meshes/humans/*")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "open3d>=0.17", "trimesh>=3.9"],
    zip_safe=True,
    maintainer="HackathonLoDT team",
    maintainer_email="team@example.com",
    description="Tools for identity roundtrip and later synthetic LiDAR generation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag_roundtrip = synthetic_data_generation.bag_roundtrip:main",
            "bag_roundtrip_smoke = synthetic_data_generation.smoke_test:main",
        ],
    },
)
