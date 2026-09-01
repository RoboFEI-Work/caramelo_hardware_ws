#!/bin/sh
# Verifica que nao sobrou nada do bringup. Sai com 0 se limpo, 1 se nao.
#
# Uso: ssh raspberrypi@<host> 'bash -s' < tools/remote/assert_clean.sh

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

FALHA=0

if pgrep -af 'ros2_control_node|ros2 launch|robot_state_publisher|ekf_node|sllidar_node|wit_ros2_imu|scan_normalizer|mesh_server|spawner' 2>/dev/null; then
	echo "FAIL processos ROS vivos"
	FALHA=1
else
	echo "PASS nenhum processo ROS"
fi

# GPIO preso e' SEMPRE sintoma de dono vivo: o kernel libera as linhas quando o
# fd fecha. Se aparecer consumidor aqui, procure o processo em /proc/*/fd — nao
# tente "liberar o GPIO".
N=$(ls -l /proc/*/fd 2>/dev/null | grep -c gpiochip)
if [ "$N" -eq 0 ]; then
	echo "PASS nenhum fd em gpiochip"
else
	echo "FAIL $N fds abertos em gpiochip"
	FALHA=1
fi

PRESAS=$(gpioinfo gpiochip4 2>/dev/null | awk '/line +(5|6|16|17|20|21|22|23|24|25|26|27):/' | grep -v unused)
if [ -z "$PRESAS" ]; then
	echo "PASS as 12 linhas do robo estao livres"
else
	echo "FAIL linhas do robo ainda reivindicadas:"
	echo "$PRESAS"
	FALHA=1
fi

if ls -l /proc/*/fd 2>/dev/null | grep -qE 'imu_usb|lidar_usb|manip_usb'; then
	echo "FAIL serial preso"
	FALHA=1
else
	echo "PASS seriais livres"
fi

if ss -lnt 2>/dev/null | grep -q ':8000'; then
	echo "FAIL porta 8000 ocupada (mesh_server zumbi)"
	FALHA=1
else
	echo "PASS porta 8000 livre"
fi

# --no-daemon faz discovery na hora, sem passar pelo cache que sustenta os
# "topicos fantasma".
NOS=$(timeout 20 ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -v '^$')
if [ -z "$NOS" ]; then
	echo "PASS grafo ROS vazio"
else
	echo "FAIL nos ainda na rede:"
	echo "$NOS"
	FALHA=1
fi

exit "$FALHA"
