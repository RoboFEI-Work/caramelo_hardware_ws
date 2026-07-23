# Relatório — Migração do controlador para Raspberry Pi 5

**Data:** 23/07/2026
**Branch:** `rasp_5` (commit `b8b719c`)
**Hardware:** Raspberry Pi 5 16 GB · SanDisk 256 GB · alimentação por power bank USB-C 45 W

## Resumo

O workspace `caramelo_hardware_ws` foi migrado da Raspberry Pi 4 (4 GB) para a Raspberry Pi 5 (16 GB), com Ubuntu Server 24.04 e ROS 2 Jazzy nativos (sem Docker). A única mudança de código necessária foi a troca do backend de GPIO do driver dos motores Maxon, porque a Pi 5 usa o chip de I/O **RP1**, incompatível com a biblioteca `pigpio` usada na Pi 4. Todo o resto do workspace compilou e roda sem alteração.

## 1. Sistema base

- **Ubuntu Server 24.04.4 LTS (arm64)** gravado via Raspberry Pi Imager; usuário `raspberrypi`.
- Correção nas fontes do apt: a imagem veio sem os pockets `noble-updates`/`noble-backports` — adicionados ao `/etc/apt/sources.list.d/ubuntu.sources` conforme recomendação da documentação oficial do ROS 2.
- **Acesso**: SSH por chave (sem senha) como `raspberrypi@raspberrypi.local`. O mDNS responder roda na própria Pi (systemd-resolved, eth0 + wlan0), então o nome `raspberrypi.local` funciona em qualquer rede/PC, inclusive cabo direto (fallback de IP link-local configurado no netplan). Compatível com o Remote Explorer do VS Code.
- **Rede atual de bancada**: cabo Ethernet direto no PC com compartilhamento de internet do Windows (ICS).

## 2. ROS 2 Jazzy

- Instalado **`ros-jazzy-desktop`** completo (RViz, demos, tutoriais) + `ros-dev-tools`, `colcon`, `rosdep` — tudo nativo, via repositório oficial (pacote `ros2-apt-source`, que mantém a configuração do repositório atualizada automaticamente).
- Stack **`ros2_control`** instalado (`ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`) + `dynamixel_sdk`, `pal_statistics`, `robot_localization`, `xacro`.
- Validação: `demo_nodes_cpp talker` ↔ `demo_nodes_py listener` comunicando via DDS.
- Ambiente carregado no `.bashrc` (`source /opt/ros/jazzy/setup.bash`).

## 3. Preparação de hardware (config.txt e permissões)

- Habilitados **I2C** (`dtparam=i2c_arm=on`), **SPI** (`dtparam=spi=on`) e **UART** (`enable_uart=1`) → `/dev/i2c-1`, `/dev/spidev0.*`, `/dev/ttyAMA0` ativos.
- **`usb_max_current_enable=1`**: libera 1,6 A nas portas USB. Necessário porque o power bank de 45 W não negocia 5 V/5 A via PD, e sem isso a Pi limita o USB a 600 mA (afetaria LiDAR/sensores USB).
- Grupos `gpio`, `i2c`, `spi` criados com regras udev; usuário em `dialout`/`video`/`gpio`/`i2c`/`spi` — **nenhum nó precisa de root** para acessar GPIO/serial.
- Timezone `America/Sao_Paulo`.

## 4. Migração do driver de motores: pigpio → lgpio

**Motivo:** a Pi 5 moveu todo o I/O do header de 40 pinos para o chip dedicado RP1; a `pigpio` (que acessava os registradores do SoC diretamente) não funciona e não será portada. A substituta é a **`lgpio`** (mesmo autor), que opera via interface `/dev/gpiochip` do kernel — sem daemon.

Mudanças no `caramelo_hardware` (isoladas em 3 arquivos):

| Antes (pigpio + daemon pigpiod) | Depois (lgpio, acesso direto) |
|---|---|
| `pigpio_start(host, port)` — dependia do daemon | `lgGpiochipOpen()` com **auto-detecção do chip RP1** pelo label (`gpiochip4` no kernel atual); parâmetro `gpiochip_device` no URDF sobrescreve |
| `set_servo_pulsewidth()` — pulsos servo aos 4 ESCs | `lgTxServo()` @50 Hz — mesma semântica de pulso (bandas 1460/1540 µs, margem de partida e mapa afim do firmware **inalterados**) |
| `notify_open/begin` + pipe `/dev/pigpioN` + thread de leitura com `poll()` | `lgGpioClaimAlert()` + callback por linha de encoder — o lgpio entrega as bordas em ordem por um thread próprio; o thread manual e o parse do pipe foram **removidos** (−123 linhas) |
| `set_pull_up_down(PI_PUD_UP)` | flag `LG_SET_PULL_UP` no próprio claim |

O que **não mudou**: tabela de decodificação de quadratura, watchdog de comando (PWM neutro se o `write()` do ros2_control parar), guarda de NaN, mapa afim pulso↔rpm dos ESCs, inversão de sinal dos motores espelhados e a interface pública do nó (o `mobile_base_hw_interface` não precisou de alteração).

**Dependência nova:** `liblgpio-dev` (apt, Ubuntu 24.04). O CMake detecta e, sem ela, compila com o backend desligado (mesmo comportamento de antes com a pigpio).

## 5. Validações realizadas

- `colcon build` do workspace completo: **4/4 pacotes**, sem warnings novos (~16 s na Pi 5).
- Biblioteca final linkada à `liblgpio.so.1` com backend habilitado.
- Smoke test em Python no ambiente real: auto-detecção do RP1 (`gpiochip4`, 54 linhas), claim + trem de pulsos servo neutro nos GPIOs 17/23/24/25 dos ESCs, claim de alerts com pull-up — tudo como usuário comum, sem sudo.
- Reboot validado com todas as interfaces (`i2c`, `spidev`, `ttyAMA0`) presentes.

## 6. Estado atual e ganhos com a Pi 5

- Workspace em `~/caramelo_hardware_ws`, branch `rasp_5`, compilando e pronto para teste em bancada com os motores.
- Com 16 GB de RAM (vs 4 GB da Pi 4): `colcon build` sem restrição de paralelismo, folga para RViz/ferramentas quando necessário, e a remoção do daemon pigpiod tira um processo de tempo-real crítico da disputa por CPU.

## 7. Pendências

1. **Teste em bancada** com os 4 motores + encoders (validar contagem de quadratura e partida dos ESCs com o novo backend).
2. **Trim de calibração por motor** (`pulse_trim_us`): proposta para corrigir a defasagem de partida observada entre motores espelhados e não-espelhados (tolerância de oscilador dos ESCs, ±22 µs) — a definir após medição em bancada.
3. **Root read-only (overlayfs)** antes de embarcar no robô, para proteger o cartão contra cortes de energia.
4. **`git push`** do commit `b8b719c` para `origin/rasp_5` (requer credenciais GitHub na Pi ou push a partir de outro clone).
