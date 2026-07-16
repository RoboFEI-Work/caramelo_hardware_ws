# Caramelo Hardware WS

Workspace ROS 2 Jazzy que roda **dentro do robo Caramelo, na Raspberry Pi**.
E ele quem liga os motores, le os encoders, o LiDAR e a IMU, e publica a
posicao do robo para o PC de operacao (`caramelo_ws`) usar.

> Se voce quer OPERAR o robo (mapa, navegacao, docking, interface grafica),
> va para o `caramelo_ws` — o README de la tem o guia para quem nao programa.
> Este workspace e o "corpo" do robo; o `caramelo_ws` e o "cerebro" de operacao.

---

## ⚠️ Avisos de seguranca (leia ANTES de energizar os motores)

1. **Perda do sinal de controle = re total.** O firmware atual dos drivers de
   motor (ESC STM32, modificado para girar nos dois sentidos) interpreta a
   AUSENCIA do sinal PWM como comando de **re na velocidade maxima**. Na
   pratica: cabo de sinal solto, Raspberry travada ou desligada com os motores
   energizados pode fazer o robo **disparar de re sozinho**.
   - **Nunca energize os motores sem a Raspberry ligada e com o bringup rodando.**
   - Testes de bancada: **rodas suspensas** (robo apoiado, rodas sem tocar o chao).
   - Detalhes e correcao recomendada: [docs/esc_stm32_comportamento_e_riscos.md](docs/esc_stm32_comportamento_e_riscos.md)
2. **Ainda nao existe botao de emergencia/contator** cortando a energia dos
   motores por hardware. Ate existir, trate o item 1 como regra de ouro.
3. "Parar" nao e instantaneo: ao comandar zero, o driver segura a ultima
   velocidade por ate meio segundo antes de parar (comportamento do firmware).

---

## Como ligar o robo

Na Raspberry (via SSH ou teclado/monitor):

```bash
cd ~/caramelo_hardware_ws
source install/setup.bash
ros2 launch raspberry_bringup hardware_bringup.launch.py
```

Isso sobe TUDO do lado do robo: motores (ros2_control), LiDAR, IMU, fusao de
odometria (EKF) e a descricao 3D do robo. Depois disso, o PC de operacao ja
"enxerga" o robo pela rede.

Para conferir se esta tudo em pe (na Pi ou no PC):

```bash
ros2 control list_controllers        # todos "active"
ros2 topic hz /scan /odom /joint_states
```

## Como atualizar o software do robo

O robo e atualizado **a mao** (nao ha deploy automatico):

```bash
cd ~/caramelo_hardware_ws
git pull
colcon build
source install/setup.bash
# e ligue de novo com o launch acima
```

---

## O que tem em cada pacote

| Pacote | Papel |
|---|---|
| `raspberry_bringup` | **Launch principal** (`hardware_bringup.launch.py`) e configuracoes (controladores a 100 Hz, LiDAR, EKF) |
| `caramelo_hardware` | Driver dos motores (leitura de encoder + PWM para os ESCs) e do braco (Dynamixel), como plugins do ros2_control |
| `caramelo_description` | Modelo 3D do robo (URDF/xacro + malhas) — fonte unica da geometria |
| `caramelo_localization` | EKF (fusao odometria + IMU) que publica a posicao local do robo |

## Documentacao tecnica

| Documento | O que ensina |
|---|---|
| [docs/calibracao_odometria.md](docs/calibracao_odometria.md) | Calibrar a odometria com trena (1 m reto, giros de 90°/360°) |
| [docs/raspberry_tempo_real.md](docs/raspberry_tempo_real.md) | Configurar prioridade de tempo real na Pi (uma vez) |
| [docs/esc_stm32_comportamento_e_riscos.md](docs/esc_stm32_comportamento_e_riscos.md) | Como o driver de motor REALMENTE funciona + bugs conhecidos do firmware |

## Fatos importantes do hardware (validados)

- Rodas mecanum de **100 mm** de diametro; distancia entre rodas 471 mm (X) e
  300 mm (Y).
- Encoder: 114688 contagens por volta de roda (validado girando a roda na mao).
- Os ESCs controlam a velocidade em malha fechada, mas **nao giram mais devagar
  que ~3,7 rad/s de roda** (limite do firmware) — o robo nao anda mais devagar
  que ~0,19 m/s.
- Braco Dynamixel: comunicacao limitada a ~25 Hz (acima disso perde pacotes) —
  ja configurado assim.

---

## Fluxo de branches (como o time desenvolve)

```
main  ← so entra o que foi VALIDADO no robo real, sem erros
  ▲
 dev  ← integracao: recebe as branches de funcao; testa-se aqui
  ▲
feature/<funcao>  ← uma branch por funcao (ex.: feature/motores,
                    feature/calibracao)
```

1. Ninguem desenvolve direto na `main`.
2. Trabalho novo: `git checkout dev && git checkout -b feature/minha-funcao`.
3. Funcao pronta → merge na `dev` → teste integrado (bancada com rodas
   suspensas primeiro!).
4. `dev` validada no robo sem erros → merge na `main`.
