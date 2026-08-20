from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("vtd_ros2_bridge"))
        / "config"
        / "vtd_bridge.param.yaml"
    )
    config = LaunchConfiguration("config")
    host = LaunchConfiguration("rdb_host")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("rdb_host", default_value="127.0.0.1"),
            Node(
                package="vtd_ros2_bridge",
                executable="vtd_bridge_node",
                name="vtd_bridge",
                output="screen",
                parameters=[config, {"rdb_host": host}],
            ),
        ]
    )
