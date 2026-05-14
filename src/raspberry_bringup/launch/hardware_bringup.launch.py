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
    localization_path = get_package_share_path('caramelo_localization')

    use_rviz = LaunchConfiguration('rviz')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baud')
    imu_frame_id = LaunchConfiguration('imu_frame_id')
    
    urdf_path = os.path.join(robot_description_path, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_bringup_path, 'config', 'caramelo_controllers.yaml')
    ekf_config = os.path.join(localization_path, 'config', 'ekf.yaml')

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
            ('/mecanum_controller/odometry', '/odom/wheel'),
        ],
    )    
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller"],
    )

    laser_driver = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[os.path.join(
                robot_bringup_path,
                "config",
                "rplidar_s2.yaml"
            )],
            output="screen"
    )

    imu_driver = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        parameters=[{
            'port': imu_port,
            'baud': imu_baud,
            'frame_id': imu_frame_id,
        }],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Inicia o RViz se true',
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/imu_usb',
            description='Porta serial da IMU WIT',
        ),
        DeclareLaunchArgument(
            'imu_baud',
            default_value='9600',
            description='Baudrate da IMU WIT',
        ),
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='imu',
            description='Frame da IMU publicada pela WIT',
        ),
        robot_state_publisher_node,
        laser_driver,
        imu_driver,
        control_node,
        joint_state_broadcaster_spawner,
        mecanum_drive_controller_spawner,
        ekf_node,
        rviz_node,
    ])
