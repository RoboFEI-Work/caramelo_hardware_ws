# Plano De Rede PC Raspberry Para Competicao

Este guia e para executar quando a Raspberry estiver conectada ao PC do Caramelo.
Objetivo: deixar PC e Raspberry no mesmo grafo ROS 2 sem sofrer interferencia de
outros robos na rede da competicao e sem interferir neles.

## 1. Dados Que Precisam Ser Anotados

Preencha antes de iniciar os testes:

```text
PC_HOSTNAME=
PC_IP=
RASPBERRY_HOSTNAME=
RASPBERRY_IP=
SSID_OU_REDE=
ROS_DOMAIN_ID_CARAMELO=
RMW_IMPLEMENTATION=
```

Sugestao inicial:

```bash
export ROS_DOMAIN_ID=67
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Use o mesmo bloco no PC e na Raspberry.

## 2. Regra De Isolamento Na Competicao

O primeiro isolamento e o `ROS_DOMAIN_ID`. Ele precisa ser exclusivo do Caramelo
na competicao. Se outro robo usar o mesmo domain, os topicos podem aparecer
misturados.

Teste:

```bash
printenv ROS_DOMAIN_ID
ros2 doctor --report
ros2 node list
ros2 topic list
```

Resultado esperado:

- PC e Raspberry usam o mesmo `ROS_DOMAIN_ID`.
- Outros robos nao aparecem em `ros2 node list`.
- Topicos de outros robos nao aparecem em `ros2 topic list`.

Se aparecer topico de outro robo, trocar `ROS_DOMAIN_ID` nos dois lados e repetir.

## 3. Perfil De Descoberta DDS

Perfil A, normal para laboratorio:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Perfil B, mais isolado para competicao:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
export ROS_STATIC_PEERS="<IP_DO_PC>;<IP_DA_RASPBERRY>"
```

Teste o Perfil A primeiro. Se houver interferencia, discovery lento, ou topicos
sumindo quando muitos robos entram na rede, testar Perfil B.

No Perfil B, confirme que apenas PC e Raspberry descobrem um ao outro:

```bash
ros2 node list
ros2 topic list
```

Se nada aparecer, voltar para `SUBNET` e validar se o formato de
`ROS_STATIC_PEERS` aceito pela instalacao esta correto.

## 4. Testes De Rede Bruta

No PC:

```bash
ping -c 20 <IP_DA_RASPBERRY>
```

Na Raspberry:

```bash
ping -c 20 <IP_DO_PC>
```

Resultado desejado:

- perda de pacote: `0%`;
- latencia media estavel;
- sem picos grandes.

Se houver perda ou picos:

- trocar para 5 GHz se estiver em 2.4 GHz;
- aproximar do roteador;
- usar roteador dedicado se permitido;
- preferir Ethernet PC roteador e Raspberry Wi-Fi;
- reduzir RViz/visualizacao na rede da competicao.

## 5. Testes De Descoberta ROS 2

Na Raspberry, subir apenas os drivers/hardware.

No PC:

```bash
ros2 node list
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /tf
ros2 topic info /tf_static
ros2 topic info /robot_description
ros2 topic info /joint_states
```

Resultado esperado:

- `/scan` aparece com publisher na Raspberry;
- `/odom` aparece com publisher na Raspberry;
- `/tf` e `/tf_static` aparecem;
- `/robot_description` aparece;
- `/joint_states` aparece;
- nenhum topico de outros robos aparece.

## 6. Testes De Frequencia E Atraso

No PC:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /tf
ros2 topic delay /scan
ros2 topic delay /odom
```

Anote:

```text
/scan hz=
/scan delay medio=
/scan delay max=
/odom hz=
/odom delay medio=
/odom delay max=
/tf hz=
```

Sinais de problema:

- `hz` oscilando muito;
- delay crescendo com o tempo;
- mensagens chegando em rajadas;
- RViz travando junto com queda de topicos;
- erro de extrapolacao em TF.

## 7. Testes De TF

No PC:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link imu_link
```

Com Nav2/AMCL:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Resultado esperado:

- TF sem erro de extrapolacao;
- transformacoes chegando continuamente;
- `odom -> base_footprint` suave, sem saltos estranhos.

## 8. Teste De Clock

No PC e na Raspberry:

```bash
date
timedatectl status
```

Resultado esperado:

- horarios muito proximos;
- NTP ativo quando possivel.

Se houver diferenca grande, corrigir antes de testar Nav2. Dessincronizacao de
clock pode parecer problema de TF, AMCL ou controller.

## 9. Teste De Comando Do PC Para A Raspberry

Com rodas suspensas ou robo em area livre.

No PC, quando o relay `Twist -> TwistStamped` estiver ativo:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic echo /mecanum_controller/reference --once
```

Teste direto no topico do `mecanum_drive_controller`:

```bash
ros2 topic pub --once /mecanum_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

Resultado esperado:

- comando chega na Raspberry;
- mecanum recebe `TwistStamped`;
- robo nao continua andando depois do comando parar.

## 10. Teste Com Bringup Completo

Na Raspberry:

```bash
ros2 launch raspberry_bringup hardware_bringup.launch.py rviz:=false
```

No PC, quando Nav2 estiver envolvido:

```bash
ros2 launch caramelo_navigation bringup.launch.py map_name:=sala_520 use_docking:=true
```

Conferir:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
ros2 topic echo /mecanum_controller/reference
ros2 topic echo /collision_monitor_state
ros2 control list_controllers
```

## 11. Se A Rede Ficar Instavel

Ordem de investigacao:

1. `ping` entre PC e Raspberry.
2. `ROS_DOMAIN_ID` igual nos dois lados e exclusivo.
3. `ros2 node list` sem robos externos.
4. `ros2 topic hz /scan` e `/odom`.
5. `ros2 topic delay /scan` e `/odom`.
6. `tf2_echo odom base_footprint`.
7. desligar RViz e repetir.
8. testar `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF` com `ROS_STATIC_PEERS`.
9. trocar Wi-Fi por roteador dedicado ou Ethernet se permitido.

## 12. Resultado Final Esperado

Antes de entrar na arena, precisa estar verdadeiro:

- PC ve todos os topicos da Raspberry.
- Raspberry recebe comandos vindos do PC.
- Nenhum topico de outro robo aparece.
- `/scan`, `/odom` e `/tf` tem frequencia estavel.
- `map -> odom` e `odom -> base_footprint` nao apresentam extrapolacao.
- `robot_description` aparece corretamente no PC.
- `joint_states` aparece corretamente no PC.
- TF do LiDAR e IMU esta correto.
- TF do manipulador esta correto.
- `mecanum_drive_controller` esta ativo.
- `twist_relay.py` gera `/mecanum_controller/reference`, se esse for o fluxo atual.
