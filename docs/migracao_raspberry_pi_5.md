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

## 7. Bancada 27-29/07/2026 — o que a migração revelou (e os consertos)

A bancada invalidou partes do port original e produziu a arquitetura atual
(commit 5d72389 + sessão 29/07):

1. **Esticamento de pulso por carga de CPU** (o problema central da Pi 5): o
   trem servo é thread de SOFTWARE; sob carga a borda de descida atrasa →
   pulso mais longo → roda de FRENTE acelera (medido 2×!) e a de RÉ freia.
   Um `colcon build` na Pi fazia rodas GIRAREM SOZINHAS. **Conserto**: RT
   completo (ver `raspberry_tempo_real.md`: limits.d + `chrt -f 50` no launch
   + governor performance + `ondemand.service` off). Pós-RT: variação ≤8% sob
   carga. REGRA: nunca buildar na Pi com o bringup no ar.
2. **Encoders — canal A com quique na descida**: captura `gpiomon` (15.516
   ciclos) mostrou 15.304 descidas DUPLAS no A da BR; com o motor energizado o
   quique vira SUBIDA fantasma → contagem 2× (validado contra marca visual:
   encoder 11,9 voltas vs 6,1 reais). Debounce de 4 µs NÃO resolve (o fantasma
   fica a meio-ciclo da subida real — indistinguível por tempo). **Conserto**:
   contar pelo canal B (limpo: 1 anomalia em 15k ciclos) — tabela de pinos
   A↔B trocada no `mobile_base_hw_interface.cpp`; o sentido já vem do comando
   (`enc_dir_`), então nada mais muda. Validação final: 4 rodas encoder ≈
   marca (~6,1 voltas), decodificação 1× a 28.672 counts/volta.
3. **Partida da FL +200 ms** (1 ciclo extra de arming do ESC, 1ª tentativa de
   partida falha): NÃO é software (sonda A/B com troca de slots) e NÃO sumiu
   com reencaixe do conector hall (29/07). Pendência de hardware: trocar a
   placa ESC FL↔outra roda p/ isolar placa×motor, ou retry rápido no firmware.
   Demais rodas partem juntas (spread 30-50 ms, ~330 ms do comando).
4. **Assimetria de regime frente/ré → trim por ramo** (chão, 3 m + trena,
   29/07): o ramo de FRENTE rodava ~8% abaixo do mapa (ré exata) → guinada de
   ~23° em 3,4 m com comando reto. Conserto: `pulse_trim_forward_us = 8.0`
   (URDF, `mobile_base.ros2_control.xacro`; o driver desloca mapa E piso de
   margem juntos). Validado: guinada −23,6°→−2,0° (frente) e −22,3°→−2,7°
   (ré); desvio lateral 56 cm→0-2 cm; rodas casadas em 1,3%; escala da
   odometria −0,5%/−1,7% vs trena. Recalibrar com o mesmo protocolo
   (`docs/calibracao_odometria.md`) se trocar ESC/firmware/pneus.
4. **`Ctrl-C` pode deixar o `ros2_control_node` vivo** segurando GPIOs — ver
   `raspberry_tempo_real.md` §problemas conhecidos.

## 8. Pendências

1. **FL: 1ª partida falha** (+200 ms) — isolar placa×motor (troca de ESC entre
   rodas) ou retry rápido pós-falha no firmware (esc.c: falha de start →
   permanecer ARMED em vez de re-armar 200 ms).
2. **PWM por hardware** (imunidade total a carga): rp1_pwm0 alcança GPIO
   12/13/18/19 mas está `disabled` e não há overlay de 4 canais de fábrica —
   exigiria overlay customizado + mover os 4 jumpers de sinal + backend sysfs
   no driver. Só se o residual de ≤8% incomodar (decisão adiada pelo operador).
3. **Root read-only (overlayfs)** antes de embarcar (proteção do cartão).
4. Encoder full-res (quadratura 4× por PIO ou halls via UART) — hoje 1× no
   canal B atende odometria; registrado para o futuro.
