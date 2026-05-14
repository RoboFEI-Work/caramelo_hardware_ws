import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_ekf_config = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "config",
        "ekf.yaml",
    )

    ekf_config = LaunchConfiguration("ekf_config")
    odom_output_topic = LaunchConfiguration("odom_output_topic")

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            ("odometry/filtered", odom_output_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ekf_config",
            default_value=default_ekf_config,
            description="Caminho do arquivo de configuracao do EKF",
        ),
        DeclareLaunchArgument(
            "odom_output_topic",
            default_value="/odom",
            description="Topico de saida da odometria filtrada",
        ),
        robot_localization,
    ])
