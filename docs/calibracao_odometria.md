# Calibração da Odometria de Rodas — Caramelo

Procedimento físico para calibrar a cinemática mecanum do robô real. Executar **depois** da correção do `read()` (feedback real de encoder, sem deadband) e **antes** de culpar EKF ou Nav2 por erros de pose.

**Regra de ouro:** todas as medições comparam a trena/fita no chão com **`/odom/wheel`** (odometria bruta das rodas), **NUNCA** com `/odom` (que já passa pelo EKF e mistura IMU).

## Pré-requisitos

- Piso plano e com boa aderência (evitar piso muito liso: roda mecanum escorrega).
- Trena, fita crepe para marcar posições e um transferidor (ou marcação de 90°/360° no chão).
- Raspberry rodando somente o bringup de hardware:
  ```bash
  ros2 launch raspberry_bringup hardware_bringup.launch.py
  ```
- Em outro terminal (Pi ou PC no mesmo `ROS_DOMAIN_ID`):
  ```bash
  ros2 topic echo /odom/wheel --field pose.pose
  ```
- Para comandar movimento, usar o joystick ou publicar diretamente:
  ```bash
  ros2 topic pub /mecanum_controller/reference geometry_msgs/msg/TwistStamped \
    '{header: {frame_id: base_footprint}, twist: {linear: {x: 0.10}}}' -r 20
  ```
  (Ctrl+C para parar; o controller zera as rodas após `reference_timeout: 0.5 s`.)

Entre cada corrida, anotar a pose inicial de `/odom/wheel` e subtrair da final (ou reiniciar o bringup para zerar).

## Etapa 0 — Sanidade do encoder (fazer PRIMEIRO, robô suspenso ou desligado dos motores)

Gire a roda dianteira esquerda **exatamente 1 volta** com a mão e observe:

```bash
ros2 topic echo /joint_states
```

- A posição de `front_left_wheel_joint` deve variar **2π ≈ 6,283 rad (± 0,05)**.
- Se variar ~π/2 (≈ 1,57), a constante de contagens por volta está 4× errada →
  corrigir `encoder_counts_per_wheel_rev` no **URDF**
  (`caramelo_description/urdf/control/mobile_base.ros2_control.xacro`), hoje
  `114688.0` = 1024 ciclos por canal × redução 28 × quadratura x4. Desde
  2026-09-01 esse valor é parâmetro e não exige recompilar C++.
- Girar bem devagar (< 0,03 rad/s): a velocidade em `/joint_states` deve ser ≠ 0
  (confirma que o deadband de leitura foi removido).
- **O sinal também é testável agora.** Na Pi 5 o sentido é MEDIDO (quadratura x4
  por amostragem), não deduzido do último comando: girando a roda no sentido de
  marcha à frente, a posição da junta tem que **crescer** — nas quatro. Medido em
  2026-09-01 com o robô sem comando nenhum: FL +6,127, FR +6,236, BL +6,154,
  BR +6,093 rad para ~1 volta à mão.
- Nota sobre a magnitude: em 8 medições independentes (4 rodas × 2 sessões) a
  contagem deu sistematicamente **2 a 3% abaixo** do esperado, nunca acima. Ou é
  parada antes da marca de forma consistente, ou a redução nominal "28:1" é
  fracionária de verdade. A Etapa 1 absorve esse erro no `wheels_radius`; se
  quiser separar as duas causas, meça 10 voltas em vez de 1.

Repetir para as 4 rodas. Só prosseguir quando as 4 estiverem corretas.

## Etapa 1 — Raio da roda (`wheels_radius`) — escala TUDO, fazer antes da rotação

1. Marcar a posição inicial do robô no chão (fita nos 2 lados da base).
2. Comandar avanço reto (+X) até percorrer ~1,000 m; medir a distância REAL com a trena (`d_real`).
3. Anotar o deslocamento reportado por `/odom/wheel` (`d_odom`).
4. Repetir 3× para frente e 3× para trás; usar a média.
5. Corrigir em `src/raspberry_bringup/config/caramelo_controllers.yaml`:

   ```
   wheels_radius_novo = wheels_radius_atual × d_real / d_odom
   ```

   (valor atual: `0.05`). Se a razão der ~4 ou ~0,25, o problema é a Etapa 0, não o raio.
6. Rebuild não é necessário para YAML; reiniciar o bringup e **repetir o teste** até erro ≤ 2 % em 1 m.

## Etapa 2 — Geometria de rotação (`sum_of_robot_center_projection_on_X_Y_axis`) — só depois da Etapa 1

1. Marcar a orientação inicial (fita alinhada com a frente do robô).
2. Comandar rotação pura no lugar: +90°, −90° e 360° completos (3 corridas de cada).
3. Comparar o yaw REAL medido no chão (`θ_real`) com o yaw de `/odom/wheel` (`θ_odom`).
4. Corrigir no mesmo YAML:

   ```
   soma_nova = soma_atual × θ_odom / θ_real
   ```

   (valor atual: `0.381` = lx 0,236 + ly 0,145). Para robôs mecanum o valor EFETIVO
   costuma ficar 5–15 % ACIMA do geométrico por causa do escorregamento dos rolos —
   esperar `soma_nova > 0.381`.
5. Reiniciar, repetir até erro ≤ 3° em 360°.
6. **Refazer 1 corrida da Etapa 1** para confirmar que o linear não regrediu.

## Etapa 3 — Lateral 1 m (+Y) — apenas validação

O controlador upstream não tem parâmetro para escorregamento lateral, então esse erro
**não é calibrável por YAML**. Medir e documentar:

1. Comandar movimento lateral puro (+Y e −Y), 1 m, 3 corridas de cada.
2. Anotar o erro percentual (tipicamente maior que o longitudinal).
3. Se o erro lateral for relevante (> 5 %), aumentar a incerteza de `vy` em
   `twist_covariance_diagonal` no YAML do controlador (2º elemento) para o EKF
   confiar menos na odometria lateral.

## Critérios de aceitação

| Teste | Erro máximo |
|---|---|
| 1 m longitudinal | ≤ 2 % |
| Rotação 360° | ≤ 3° |
| 1 m lateral | documentar (sem knob de correção) |

## Depois da calibração

1. Validar o EKF: dirigir um percurso conhecido e comparar `/odom` com o real.
2. Só então testar Nav2 completo (PC + Pi): goals curtos, laterais, rotações e
   convergência final XY/yaw (tolerâncias atuais: 0,05 m / 0,12 rad).
