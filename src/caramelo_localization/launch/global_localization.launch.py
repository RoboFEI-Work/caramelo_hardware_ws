import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")
    use_keepout = LaunchConfiguration("use_keepout")

    local_localization_launch = os.path.join(
        get_package_share_directory("caramelo_localization"),
        "launch",
        "local_localization.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Usa clock simulado se true",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="sala_520",
            description="Nome do mapa dentro de caramelo_mapping/maps/",
        ),
        DeclareLaunchArgument(
            "use_keepout",
            default_value="false",
            description="Compatibilidade com o launch do workspace principal",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_localization_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "map_name": map_name,
            }.items(),
        ),
    ])
