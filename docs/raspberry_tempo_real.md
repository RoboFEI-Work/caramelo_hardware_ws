# Configuração de Tempo Real no Raspberry Pi (Caramelo)

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

## Opcional: governor de CPU em performance

Evita que o kernel abaixe o clock durante a operação (reduz jitter):

```bash
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

## Problemas conhecidos (não corrigir localmente)

- **`wit_ros2_imu` lança traceback no Ctrl+C** (`TypeError: 'NoneType' object cannot
  be interpreted as an integer` + `RCLError: failed to shutdown`). É um bug de
  encerramento do pacote de terceiros (`~/ros2_ws`): a thread de leitura serial
  corre contra o fechamento da porta, e o `main()` chama `rclpy.shutdown()` num
  contexto já finalizado. **Inofensivo** — acontece só no desligamento, não afeta
  a operação. NÃO editar o pacote localmente: ele é instalado do GitHub e é o
  mesmo em todos os PCs/robôs. Se incomodar, abrir PR no repositório upstream.
