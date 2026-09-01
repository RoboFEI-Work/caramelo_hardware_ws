# Preambulo de ambiente ROS 2 para execucao NAO-INTERATIVA na Raspberry.
#
# POR QUE ISTO EXISTE: o ~/.bashrc do Ubuntu comeca com
#   case $- in *i*) ;; *) return;; esac
# ou seja, ele RETORNA antes de chegar no `source /opt/ros/jazzy/setup.bash`
# quando o shell nao e' interativo. Em `ssh host 'comando'` e em unidades do
# systemd, portanto, o comando `ros2` simplesmente nao existe — e o sintoma
# ("command not found") e' facil de confundir com "o no nao subiu".
#
# Uso:  . "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

export LC_ALL=C

. /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && . "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/caramelo_hardware_ws/install/setup.bash" ] && . "$HOME/caramelo_hardware_ws/install/setup.bash"

# Ordem acima e' obrigatoria: /opt/ros (sistema) -> ros2_ws (terceiros:
# sllidar_ros2, wit_ros2_imu) -> caramelo_hardware_ws (projeto). O ultimo
# sourced tem prioridade. As tres linhas explicitas sao defesa em profundidade
# contra um prefix chain do colcon gravado sem o overlay de terceiros — nesse
# caso o launch aborta com "package not found" e parece falha do LiDAR/IMU.

export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-67}"

# O launch remove ROS_LOCALHOST_ONLY porque o Jazzy avisa mesmo com valor 0.
unset ROS_LOCALHOST_ONLY

CARAMELO_WS="$HOME/caramelo_hardware_ws"
CARAMELO_LOGS="$HOME/caramelo_logs"
CARAMELO_PGID_FILE="$CARAMELO_LOGS/bringup.pgid"
mkdir -p "$CARAMELO_LOGS"
export CARAMELO_WS CARAMELO_LOGS CARAMELO_PGID_FILE
