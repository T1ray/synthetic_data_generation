"""Start RViz2 with the packaged synthetic LiDAR preview configuration."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("synthetic_data_generation")) / "rviz" / "synthetic_lidar.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_config),
        Node(package="rviz2", executable="rviz2", name="synthetic_lidar_rviz",
             arguments=["-d", LaunchConfiguration("rviz_config")], output="screen"),
    ])
