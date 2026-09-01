#!/bin/sh
# Sobe o bringup de forma que sobreviva ao fim da sessao SSH e possa ser
# encerrado limpo depois.
#
# Uso (do PC, por stdin — NUNCA por argv, ver nota de path conversion abaixo):
#   ssh raspberrypi@<host> 'bash -s' -- use_lidar:=false use_imu:=false \
#       < tools/remote/run_bringup.sh
#
# NOTA sobre argv: no Git Bash do Windows, um argumento como /opt/ros/... e'
# convertido para caminho Windows antes de sair da maquina. Por isso todo
# script deste diretorio e' enviado por STDIN, com heredoc de delimitador entre
# aspas simples.

. "$HOME/caramelo_hardware_ws/tools/lib/env.sh"

# Guarda contra dois donos do mesmo hardware. Subir um segundo
# robot_state_publisher e' especialmente traicoeiro: ele publica o URDF num
# topico latched (TRANSIENT_LOCAL) e o controller_manager novo pode engolir a
# descricao ANTIGA sem nenhum erro — parametros novos do xacro simplesmente
# "nao fazem efeito".
if pgrep -f 'ros2_control_node|robot_state_publisher' >/dev/null 2>&1; then
	echo "ABORTANDO: ja ha processos ROS no ar. Rode stop_bringup.sh antes."
	pgrep -af 'ros2_control_node|robot_state_publisher' | head -5
	exit 3
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG="$CARAMELO_LOGS/bringup_$RUN_ID.log"

# setsid: nova sessao, entao o processo NAO recebe SIGHUP quando o SSH cai, e
# ganha um process group proprio — que e' o alvo do SIGINT no stop_bringup.
# O `echo $$` de dentro do bash -c grava o PGID de forma deterministica; o $!
# do shell pai nao serve, porque sem job control o filho pode herdar o grupo
# do shell.
# </dev/null e a redirecao sao o que faz o ssh RETORNAR em vez de pendurar
# esperando o canal fechar.
setsid bash -c '
  echo $$ > "'"$CARAMELO_PGID_FILE"'"
  exec ros2 launch raspberry_bringup hardware_bringup.launch.py '"$*"'
' </dev/null >"$LOG" 2>&1 &

sleep 2
echo "RUN_ID=$RUN_ID"
echo "PGID=$(cat "$CARAMELO_PGID_FILE" 2>/dev/null)"
echo "LOG=$LOG"
