# APOSENTADO — NAO USE ESTE LAUNCH.
#
# Este arquivo era uma versao antiga "open-loop, sem EKF" do bringup. Ele
# remapeava a TF e a odometria do mecanum_controller DIRETO para /tf e /odom:
#     ('/mecanum_controller/tf_odometry', '/tf')
#     ('/mecanum_controller/odometry',    '/odom')
# Isso faz o CONTROLADOR virar dono do TF odom->base_footprint e de /odom,
# entrando em conflito frontal com o bringup real (raspberry_bringup/
# hardware_bringup.launch.py), onde o EKF e' o unico dono de /odom e desse TF,
# e o controlador vai para /odom/wheel. Rodar este launch (sozinho ou junto)
# cria DOIS publishers de /odom e TF duplicada -> odometria corrompida.
#
# Alem disso, ele gera o URDF sem os args use_manipulator/use_realsense/
# visual_mode:=http e nao sobe IMU nem LiDAR.
#
# USE SEMPRE:  ros2 launch raspberry_bringup hardware_bringup.launch.py
#
# Mantido no repositorio apenas por historico; aborta de proposito para evitar
# que seja usado por engano (auditoria Fase 0 / RK-01).

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    raise RuntimeError(
        "\n[CARAMELO] hardware_interface.launch.py foi APOSENTADO.\n"
        "Ele remapeia TF/odom do controlador direto para /tf e /odom, conflitando "
        "com o EKF do bringup real.\n"
        "Use:  ros2 launch raspberry_bringup hardware_bringup.launch.py\n"
    )
