# Configuração de Tempo Real no Raspberry Pi (Caramelo)

> ⚠️ **OBRIGATÓRIO EM CADA Pi NOVA/REINSTALADA — não migra com o workspace!**
> Na troca rasp4→rasp5 (07/2026) esta config ficou para trás e o resultado foi
> grave: na Pi 5 os pulsos servo dos ESCs são gerados por thread de SOFTWARE
> (lgpio; a pigpio/DMA não existe mais), e sem RT a borda de descida ATRASA sob
> carga de CPU → pulso mais longo → roda de FRENTE acelera (~2×!) e a de RÉ
> desacelera. Provado em bancada 27-29/07/2026 (comando 3,6 rad/s: direitas
> foram a 8-9 rad/s durante um `colcon build`). Com RT aplicado: variação ≤8%.
> Status: **aplicado na rasp5 em 2026-07-29** (incl. `chrt -f 50` no launch).

O `controller_manager` tenta criar a thread de controle com política FIFO e prioridade 50.
Sem permissão, aparece no bringup:

```
[WARN] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted).
```

Sem RT o loop de 100 Hz funciona, mas fica sujeito a jitter quando o sistema carrega
(rede, EKF, LiDAR). A configuração abaixo é feita **uma única vez, à mão, no Pi**.

## Passo a passo (uma vez, no Raspberry)

1. Criar o arquivo de limites (usuário `raspberrypi`):

   ```bash
   sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<'EOF'
   raspberrypi - rtprio 98
   raspberrypi - memlock unlimited
   EOF
   ```

2. Reiniciar o Pi (ou fazer logout/login completo da sessão SSH):

   ```bash
   sudo reboot
   ```

3. Verificar depois do login:

   ```bash
   ulimit -r   # deve mostrar 98
   ```

4. Rodar o bringup e confirmar que o WARN de "FIFO RT scheduling" sumiu:

   ```bash
   ros2 launch raspberry_bringup hardware_bringup.launch.py
   ```

## Governor de CPU em performance (recomendado, feito na rasp5)

Evita que o kernel abaixe o clock durante a operação (reduz jitter). Sem
internet na Pi, sem cpufrequtils — usar serviço systemd próprio (foi o feito
em 2026-07-29):

```bash
sudo tee /etc/systemd/system/cpufreq-performance.service > /dev/null <<'EOF'
[Unit]
Description=CPU governor performance (pulsos servo lgpio estaveis)
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for c in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > $c; done'

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload && sudo systemctl enable --now cpufreq-performance.service
# Ubuntu tem um servico legado que RESETA para ondemand depois do boot:
sudo systemctl disable --now ondemand.service
```

## chrt no launch (já commitado)

O `hardware_bringup.launch.py` sobe o `ros2_control_node` com
`prefix=["chrt -f 50"]` — o processo INTEIRO (incl. threads internas do lgpio
que geram os pulsos e leem encoders) roda SCHED_FIFO 50. Depende do limits.d
acima; sem ele o chrt falha e o launch quebra (proposital: rodar este doc
primeiro).

## Problemas conhecidos (não corrigir localmente)

- **Ctrl-C no bringup às vezes NÃO mata o `ros2_control_node`** (observado
  2026-07-29 com FIFO): o processo fica vivo segurando os GPIOs ("Device or
  resource busy" p/ qualquer outro uso). Conferir com
  `ps -eo pid,comm | grep ros2_control` e matar com `kill <pid>` — o driver
  tem shutdown ordenado (neutro no fio → corta pulsos) e o firmware dos ESCs
  para os motores sozinho se os pulsos sumirem.

- **`wit_ros2_imu` lança traceback no Ctrl+C** (`TypeError: 'NoneType' object cannot
  be interpreted as an integer` + `RCLError: failed to shutdown`). É um bug de
  encerramento do pacote de terceiros (`~/ros2_ws`): a thread de leitura serial
  corre contra o fechamento da porta, e o `main()` chama `rclpy.shutdown()` num
  contexto já finalizado. **Inofensivo** — acontece só no desligamento, não afeta
  a operação. NÃO editar o pacote localmente: ele é instalado do GitHub e é o
  mesmo em todos os PCs/robôs. Se incomodar, abrir PR no repositório upstream.
