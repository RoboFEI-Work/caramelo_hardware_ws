# ESC STM B-G431B-ESC1 — Comportamento Real e Riscos (análise do firmware)

Análise do projeto `Projeto_com_hall_final_22_06_2026` (MCSDK 6.4.1, Motor Control
Workbench + CubeMX, modificado pela equipe para girar nos dois sentidos).
**Nenhuma alteração foi feita no firmware** — este documento só registra o que o
código faz de verdade, para o driver da Raspberry conversar corretamente com ele.

## 1. Como o ESC interpreta o pulso PWM (verificado no código)

- O ESC roda **controle de velocidade em malha fechada** (FOC, modo velocidade,
  PI de velocidade a 1 kHz com feedback dos sensores hall) — `drive_parameters.h:98`,
  `esc.c` usa `MCI_ExecSpeedRamp()`.
- O mapa pulso → velocidade é **AFIM, com um piso**, não proporcional
  (`esc.c:230`, frente; `esc.c:332`, ré):

  ```
  rpm_motor = 1000 + (pulso_us − 1520) × (5364 − 1000) / 480      (frente)
  rpm_motor = 1000 + (1480 − pulso_us) × (5364 − 1000) / 480      (ré, sinal invertido)
  ```

- Em rad/s de RODA (gearbox 1:28):
  - **Piso: 1000 rpm = 3,74 rad/s** — o ESC NÃO executa velocidade menor que isso
    (`speed_min_valueRPM` fixo em `mc_parameters.c:195`). A roda "salta" de 0 para
    3,74 rad/s na borda da banda (1520/1480 µs).
  - **Escala cheia: 5364 rpm = 20,06 rad/s** em 2000/1000 µs.
- Faixas de pulso (`parameters_conversion_g4xx.h:50-58`): neutro sem armar
  1490–1510 µs; frente ativa 1520–2000 µs; ré ativa 1480–1000 µs. Armar frente
  demora 200 ms (`ARMING_TIME`); ré arma instantaneamente (bug, ver §3).
- Resposta a mudanças de referência: rampa de ~50 ms (`esc.c:238/340`).
- O teto medido de ~10,3 rad/s (2754 rpm) NÃO é limite do firmware (que permite
  5364): é saturação física — clamp de torque `IQMAX = 4 A` e/ou limite de tensão
  do barramento (Ke = 3,3 Vrms/kRPM precisa de ~25 V para 5364 rpm; bateria de 24 V
  caindo sob carga satura antes).

## 2. Consequências para o robô

- O mapa antigo do driver (proporcional, 0→21,3 rad/s) enviava 1610 µs para um
  comando de 4 rad/s; o firmware executava 1819 rpm = 6,8 rad/s — bate com os
  6,3 rad/s medidos. **Corrigido no driver** (mapa afim inverso).
- **Velocidade mínima executável por roda: 3,74 rad/s** ≈ 0,19 m/s de translação
  ou ≈ 0,49 rad/s de rotação pura do robô. Manobras mais lentas que isso são
  FISICAMENTE impossíveis com este firmware — o robô "chicoteia" nas rotações
  lentas porque não existe rotação lenta. O driver agora aplica a política
  "mais próximo executável" (< meio piso → parado; ≥ meio piso → piso), mas a
  solução real é reduzir `speed_min_valueRPM` no firmware (ver §4).
- **Neutro não é freio**: em movimento, pulso neutro mantém a última referência
  por ~0,5 s (`TURNOFF_TIME_MAX`) antes de parar o motor, e a parada é por
  inércia. O robô continua andando ~0,5 s depois de comandar zero.

## 3. Riscos encontrados no firmware (REPORTAR — não alterado)

1. **CRÍTICO — perda do sinal PWM = ré total.** O watchdog de sinal força
   `Ton_value = 0` após ~0,5 s sem bordas (`esc.c:104-116`). Com a modificação de
   ré, `Ton = 0` é interpretado como pulso mínimo → **ré em velocidade máxima
   (5364 rpm ≈ 20 rad/s de roda)**: de `ESC_ARMING`, `Ton=0` arma ré na hora e dá
   partida (`esc.c:145, 271-282`); em `ESC_NEGATIVE_RUN`, vira referência máxima
   (`esc.c:332-340`). **Na prática: desconectar o cabo de sinal, travar a
   Raspberry/pigpiod ou desligar a Pi com os ESCs energizados pode disparar
   fuga em ré total.** Até o firmware ser corrigido: nunca energizar os motores
   sem a Pi rodando o bringup; testes de bancada só com as rodas suspensas.
2. **Armar ré não tem tempo de armação** — comparação errada em `esc.c:147`
   (int32 promovido a unsigned): ré arma no primeiro tick de 1 ms, frente espera
   200 ms. Assimetria de partida entre os sentidos.
3. **Falha auto-reconhecida**: qualquer `FAULT_OVER` (sobrecorrente etc.) é limpo
   automaticamente e o ESC rearma sozinho (`esc.c:368-380`) — com sinal não-neutro
   presente, o motor religa sozinho após uma falha.
4. **Base de tempo errada nos comentários**: `esc.c` assume hook de 400 Hz, mas o
   hook real é 1 kHz — todas as durações nomeadas são 2,5× mais rápidas do que os
   comentários dizem (`STOP_DURATION` = 100 ms, não 2 s).
5. **A modificação de ré foi feita dentro da biblioteca do SDK**
   (`MCSDK_v6.4.1-Full/.../Any/Src/esc.c`), não em código do projeto: regenerar o
   projeto no Workbench ou reinstalar o MCSDK **reverte a modificação em silêncio**.
6. Código morto/comentários obsoletos: estado `ESC_WAIT_FOR_STOP` nunca entra,
   globais `tmpRpm/speedDec/...` sem uso, comentário de limiar de ré desatualizado
   (1450 vs 1480 µs).

## 4. Mudanças de firmware recomendadas (quando a equipe decidir mexer)

Em ordem de prioridade:

1. **Corrigir o failsafe de perda de PWM** para parar o motor (estado seguro),
   nunca interpretar `Ton=0` como ré máxima.
2. **Reduzir `speed_min_valueRPM`** (hoje 1000 rpm): com hall + FOC dá para rodar
   bem mais baixo (testar 100–300 rpm → piso de 0,37–1,1 rad/s de roda). É a
   mudança que destrava manobras lentas e docking de precisão.
3. **Reduzir `TURNOFF_TIME_MAX`** (hoje ~0,5 s segurando a última velocidade em
   neutro) para ~50 ms, para o robô parar quando o Nav2 manda parar.
4. Corrigir o tempo de armação da ré e mover a modificação para código do projeto
   (fora da árvore do MCSDK).

Depois de QUALQUER mudança no firmware, recalibrar `min/max_wheel_rad_per_sec`
no URDF (`mobile_base.ros2_control.xacro`) — o driver lê esses valores de lá.

## 5. Calibração fina do mapa (na Raspberry, rodas suspensas)

O mapa afim do driver usa os valores nominais do firmware (3,74 / 20,06). O ponto
medido (4 → 6,3 rad/s) sugere inclinação real ~13 % menor que a nominal (carga/
tensão da bateria). Para refinar:

1. Rodas suspensas, bringup rodando, bateria carregada.
2. Comandar velocidades fixas de roda e anotar a média de `maxon/wheel_velocity`
   (usar média de vários segundos — o tópico é ruidoso a 100 Hz).
3. Ajustar no ponto médio da faixa útil: se o comando 6 rad/s executa X, então
   `max_wheel_rad_per_sec_novo = piso + (X − piso) × (20,06 − 3,74) / (6 − 3,74)`…
   ou simplesmente ajustar `max_wheel_rad_per_sec` até comando ≈ medido na faixa
   4–8 rad/s. Editar SÓ o URDF (`<param name="max_wheel_rad_per_sec">`) e
   reconstruir `caramelo_description` (sem recompilar C++).
4. Repetir para a ré (deve ser espelhada).
