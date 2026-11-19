"""Kinect v1 driver.

    ros2 launch kinect_v1_ros2 kinect_v1.launch.py [simulate:=true] [namespace:=kinect_v1]
    Second Kinect: namespace:=kinect_flur device_index:=1 (frames kinect_flur_...)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("kinect_v1_ros2"), "config", "kinect_v1.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulate", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="kinect_v1"),
            DeclareLaunchArgument("params", default_value=params),
            DeclareLaunchArgument("device_index", default_value="0"),
            DeclareLaunchArgument(
                "frame_prefix", default_value=LaunchConfiguration("namespace")
            ),
            Node(
                package="kinect_v1_ros2",
                executable="kinect_v1_node",
                name="driver",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params"),
                    {
                        "simulate": LaunchConfiguration("simulate"),
                        "device_index": LaunchConfiguration("device_index"),
                        "frame_prefix": LaunchConfiguration("frame_prefix"),
                    },
                ],
                respawn=True,
                respawn_delay=3.0,
            ),
        ]
    )
