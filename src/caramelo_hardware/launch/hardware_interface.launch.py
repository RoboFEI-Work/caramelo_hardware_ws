from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('caramelo_description')
    robot_bringup_path = get_package_share_path('raspberry_bringup')
    
    urdf_path = os.path.join(robot_description_path, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'caramelo_controllers.yaml')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ('/mecanum_controller/tf_odometry', '/tf'),
            ('/mecanum_controller/odometry', '/odom'),
        ],
    )    
    

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
    ])
