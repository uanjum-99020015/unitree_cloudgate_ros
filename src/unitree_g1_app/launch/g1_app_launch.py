#!/usr/bin/env python3
"""Launch movement, audio, and video nodes for Unitree G1 (Ubuntu 24.04, ROS2 Jazzy)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("unitree_g1_app")
    config_path = os.path.join(pkg_share, "config", "g1_app_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config_path, description="Path to params file"),
        Node(
            package="unitree_g1_app",
            executable="movement_node",
            name="movement_node",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
        Node(
            package="unitree_g1_app",
            executable="audio_node",
            name="audio_node",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
        Node(
            package="unitree_g1_app",
            executable="video_node",
            name="video_node",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
    ])
