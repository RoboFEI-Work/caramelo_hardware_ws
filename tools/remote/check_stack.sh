#!/bin/sh
# Verificacao de saude do bringup no ar, com criterios objetivos.
#
# Uso: ssh raspberrypi@<host> 'bash -s' < tools/remote/check_stack.sh

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

echo "===== componentes de hardware ====="
timeout 25 ros2 control list_hardware_components 2>&1 | head -10

echo "===== controladores (todos devem estar active) ====="
timeout 25 ros2 control list_controllers 2>&1 | head -10

echo "===== /robot_description: Publisher count DEVE ser 1 ====="
# Mais de um publisher = robot_state_publisher zumbi servindo URDF velho num
# topico latched. O controller_manager pode engolir a descricao antiga sem
# nenhum erro, e ai "parametro novo do xacro nao faz efeito".
timeout 20 ros2 topic info /robot_description -v 2>/dev/null | grep -i "publisher count"

echo "===== prioridade do ros2_control_node (esperado SCHED_FIFO 50) ====="
CM=$(pgrep -f ros2_control_node | head -1)
[ -n "$CM" ] && chrt -p "$CM" 2>/dev/null

echo "===== taxas ====="
echo "-- /joint_states (esperado ~100 Hz) --"
timeout -s INT 10 ros2 topic hz /joint_states 2>&1 | tail -2
echo "-- /odom/wheel (esperado ~100 Hz, mesmo SEM comando) --"
timeout -s INT 10 ros2 topic hz /odom/wheel 2>&1 | tail -2
echo "-- /maxon/wheel_velocity (telemetria, esperado ~20 Hz) --"
timeout -s INT 8 ros2 topic hz /maxon/wheel_velocity 2>&1 | tail -2

echo "===== TF odom -> base_footprint ====="
timeout 12 ros2 run tf2_ros tf2_echo odom base_footprint --once 2>&1 | head -8

echo "===== recursos ====="
uptime
free -m | head -2
echo -n "temperatura: "; awk '{printf "%.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null
echo -n "governor: "; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
ps -eo pid,pri,rtprio,pcpu,rss,comm --sort=-pcpu 2>/dev/null | head -6
