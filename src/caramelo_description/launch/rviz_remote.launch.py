from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = os.path.join(
        get_package_share_path('caramelo_description'),
        'rviz',
        'urdf_config.rviz',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Inicia RViz usando o ambiente local do caramelo_description',
        ),
        rviz_node,
    ])
