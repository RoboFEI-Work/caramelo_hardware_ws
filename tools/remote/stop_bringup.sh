#!/bin/sh
# Encerramento do bringup em estagios, do mais limpo para o mais bruto.
#
# O estagio 1 (SIGINT no GRUPO de processos) e' exatamente o que um Ctrl-C faz
# num terminal, e e' o unico caminho que executa o shutdown ORDENADO do driver:
# neutro -> 120 ms no fio (>=5 pulsos, o firmware ve o neutro de verdade) ->
# corta os pulsos -> libera as linhas. Um SIGKILL pula tudo isso.
#
# Uso: ssh raspberrypi@<host> 'bash -s' < tools/remote/stop_bringup.sh

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

PGID="$(cat "$CARAMELO_PGID_FILE" 2>/dev/null)"

vivo() { pgrep -f 'ros2_control_node|ros2 launch|robot_state_publisher' >/dev/null 2>&1; }

if [ -n "$PGID" ]; then
	echo "estagio 1: SIGINT no grupo -$PGID"
	kill -INT -"$PGID" 2>/dev/null
	i=0
	while [ "$i" -lt 15 ]; do
		vivo || { echo "  saiu limpo em ${i}s"; break; }
		i=$((i + 1))
		sleep 1
	done
fi

if vivo; then
	echo "estagio 2: SIGTERM"
	[ -n "$PGID" ] && kill -TERM -"$PGID" 2>/dev/null
	sleep 5
fi

if vivo; then
	# PERIGO: SIGKILL pula o shutdown ordenado e os pulsos somem de uma vez.
	# So' faca isso com a alimentacao dos motores DESLIGADA e rodas suspensas.
	echo "estagio 3: SIGKILL — confirme que os motores estao DESENERGIZADOS"
	[ -n "$PGID" ] && kill -KILL -"$PGID" 2>/dev/null
	sleep 2
fi

echo "estagio 4: varredura por nome"
for padrao in ros2_control_node robot_state_publisher ekf_node sllidar_node \
	wit_ros2_imu scan_normalizer mesh_server.py 'controller_manager.*spawner' 'ros2 launch'; do
	pkill -f "$padrao" 2>/dev/null
done
sleep 2

# O daemon do ros2 cacheia o grafo: depois de matar nos, `ros2 node list` e
# `ros2 topic list` CONTINUAM listando os mortos por um tempo indeterminado —
# e e' exatamente isso que vira "topico fantasma" na percepcao de quem opera.
# `daemon stop` limpa o cache; nao mata no nenhum.
echo "estagio 5: limpando o cache do daemon"
ros2 daemon stop >/dev/null 2>&1

rm -f "$CARAMELO_PGID_FILE"
echo "encerrado"
