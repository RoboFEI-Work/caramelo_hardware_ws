from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('caramelo_description'),
                             'urdf', 'robots', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('caramelo_description'),
                                    'rviz', 'urdf_config.rviz')

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_manipulator = LaunchConfiguration('use_manipulator')
    use_realsense = LaunchConfiguration('use_realsense')
    use_mock_components = LaunchConfiguration('use_mock_components')
    manip_mount_xyz = LaunchConfiguration('manip_mount_xyz')
    manip_mount_rpy = LaunchConfiguration('manip_mount_rpy')
    
    robot_description = ParameterValue(
        Command([
            'xacro ', model,
            ' use_manipulator:=', use_manipulator,
            ' use_realsense:=', use_realsense,
            ' use_mock_components:=', use_mock_components,
            ' manip_mount_xyz:="', manip_mount_xyz, '"',
            ' manip_mount_rpy:="', manip_mount_rpy, '"',
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=urdf_path,
            description='Caminho do arquivo URDF/Xacro principal',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Inicia joint_state_publisher_gui se true',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Inicia RViz se true',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usa clock de simulacao se true',
        ),
        DeclareLaunchArgument(
            'use_manipulator',
            default_value='true',
            description='Inclui o manipulador na descricao completa',
        ),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Inclui a RealSense no manipulador se true',
        ),
        DeclareLaunchArgument(
            'use_mock_components',
            default_value='true',
            description='Usa mock_components para o ros2_control do manipulador',
        ),
        DeclareLaunchArgument(
            'manip_mount_xyz',
            default_value='0.185 0 0.078',
            description='AJUSTAR_NO_ROBO: posicao do manipulador em relacao ao base_link',
        ),
        DeclareLaunchArgument(
            'manip_mount_rpy',
            default_value='0 0 0',
            description='AJUSTAR_NO_ROBO: orientacao do manipulador em relacao ao base_link',
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
