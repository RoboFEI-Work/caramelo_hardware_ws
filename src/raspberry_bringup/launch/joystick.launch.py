import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_teleop_path = os.path.join(get_package_share_directory("raspberry_bringup"), "config", "joy_teleop.yaml")
    joy_config_path = os.path.join(get_package_share_directory("raspberry_bringup"), "config", "joy_config.yaml")

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[joy_teleop_path]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[joy_config_path]
    )

    return LaunchDescription(
        [
            joy_teleop,
            joy_node
        ]
    )
