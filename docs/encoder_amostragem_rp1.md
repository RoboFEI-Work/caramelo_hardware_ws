# Encoder na Raspberry Pi 5: quadratura x4 por amostragem do RIO

Documento da arquitetura de leitura de encoder adotada em 2026-09-01, com as
medições que a justificam. Substitui a decodificação 1× por eventos do lgpio que
foi usada de julho a agosto de 2026.

## O problema que forçou a mudança

O requisito novo era simples de enunciar: **o encoder tem que contar certo com o
robô sendo arrastado à mão, sem comando nenhum**, para dar para ajustar a pose do
robô empurrando ele e ver o TF acompanhar.

Na arquitetura anterior isso era impossível por construção. A contagem era 1×
(só bordas de subida de uma linha) e o **sentido vinha do último comando de pulso
não-neutro** (`enc_dir_`). Sem comando, o sentido ficava congelado no último
valor — ou em `+1`, o valor inicial. Empurrar o robô para trás contava para
frente.

Esse desenho não era descuido: ele resolveu um problema real. A tentativa
anterior lia o nível do canal irmão dentro do callback, e a ~27 mil eventos/s por
roda a fila de alertas atrasava, o nível lido já havia mudado e o sinal de cada
contagem virava aleatório (comando de +6 rad/s medindo −2 a −4 rad/s).

## Por que a solução não podia ser "consertar o evento"

Medir sentido exige observar **os dois canais**. Observar dois canais por evento
custa `4·f` eventos/s por roda — e esse custo é o mesmo contando x1, x2 ou x4.
Ou seja, **x1 não economiza nada; só joga a informação de sentido fora.**

| ω roda | ciclos/canal | eventos x4, 4 rodas | Δt mínimo entre transições |
|---|---|---|---|
| 2,43 rad/s (piso) | 11,1 kHz | 177 k/s | 22,5 µs |
| 6 rad/s (cruzeiro) | 27,4 kHz | 438 k/s | 9,1 µs |
| 20,06 rad/s (escala cheia) | 91,6 kHz | 1,46 M/s | 2,73 µs |

Foi exatamente essa conta que matou a quadratura em userspace em julho.

**A conclusão é de arquitetura, não de implementação:** se o custo por evento é
proibitivo, a leitura não pode ser orientada a evento. Tem que ser orientada a
**amostragem**, onde o custo é constante e independe da velocidade da roda.

## A solução

Uma leitura MMIO de 32 bits no registrador `RIO_IN` do RP1 devolve o nível de
GPIO0..27 **simultaneamente** — os 8 canais de encoder do robô numa única
transação. Acesso por `mmap` de `/dev/gpiomem0`, sem root (basta o grupo `gpio`).

Uma thread dedicada amostra em laço fechado e alimenta uma máquina de estados de
quadratura x4. O `read()` do `ros2_control` consome um instantâneo por seqlock.

### Medições na Pi 5 (Ubuntu 24.04, kernel 6.8.0-1060-raspi)

| item | medido |
|---|---|
| leitura MMIO isolada | ~160 ns |
| taxa com leitura em rajada | **5,43 MHz** |
| taxa lendo e ramificando a cada amostra | 1,04 MHz |
| amostras por transição na velocidade máxima | ~15 |

A leitura em **rajada** importa: ler e ramificar em cada amostra deixa o core
parado esperando a transação PCIe voltar (~960 ns de latência efetiva). Emitindo
as leituras em rajada, várias ficam em voo e a taxa sobe 5×.

## O efeito colateral que resolveu um bug antigo

Havia um bug aberto e sem documentação: roda parada em cima de uma borda óptica
faz o comparador do encoder chilrear (~60 kHz medidos), e como o sentido vinha do
comando, o chilrear integrava numa direção só — **fantasma de 13 rad/s com o robô
parado**, o robô "passeando" no RViz. A tentativa de conserto foi um "portão de
repouso" (commit `1f42cc6`) que descartava contagens após 800 ms em neutro, e foi
revertida em `f23f5dc` porque fazia sumir a inércia pós-comando e impedia contar
empurrões manuais.

Com quadratura isso **se resolve sozinho**: no código Gray `00 → 01 → 11 → 10`,
uma linha oscilando sozinha produz `+1, −1, +1, −1…`. A soma é identicamente zero
e o erro instantâneo fica limitado a ±1 count = ±2,7 µm de arco.

Medido com os motores **energizados** e o robô parado por 30 s: posição das
quatro juntas exatamente `0.0`. Em 60 s: 1 count numa roda, que não mudou depois.

## Filtro de permanência

Girando as rodas à mão, o decodificador acusou transições **ilegais** (as duas
linhas mudando dentro da mesma janela de 184 ns): 1709 na BR, 183 na FR. Isso não
é perda de amostragem — a 5,4 MHz há ~124 amostras por transição legítima na
velocidade de mão. É qualidade de sinal: ringing e diafonia entre os fios A e B
do mesmo chicote, na mesma roda que a bancada de julho já tinha apontado como a
pior (15304 descidas duplas em 15516 ciclos).

O padrão dos dados diz de onde vem: **cada roda só produz ilegais enquanto o
motor dela está girando** — é o acoplamento das fases do motor no chicote do
próprio encoder.

A correção é exigir que uma palavra dure N amostras antes de ser aceita:

| filtro | FL | FR | BL | BR |
|---|---|---|---|---|
| `--stable 1` | 232 ilegais | 0 | 0 | 194 ilegais |
| `--stable 8` | **0** | **0** | **0** | **0** |

Sem perder nenhuma contagem legítima. O teto é físico: na velocidade máxima o
intervalo mínimo entre transições legítimas é 2,73 µs (~15 amostras), e 8
amostras são ~1,5 µs — fator 1,9 de margem no pior caso operacional.

## Requisitos de sistema (todos no `tools/setup_pi.sh`)

- **`isolcpus=3 nohz_full=3 rcu_nocbs=3`** no `cmdline.txt`. A thread consome um
  core inteiro por projeto; sem isolamento ela compete com o resto e o load geral
  dobra (medido: 3,0 sem isolamento contra 1,43 com).
- **`kernel.sched_rt_runtime_us = -1`**. Com o padrão (950000/1000000), uma thread
  `SCHED_FIFO` que usa 100% de um core leva um **blackout de 50 ms a cada
  segundo** — medido: pior intervalo entre amostras 49,99 ms com throttling
  ligado, 16 µs sem. Isso viraria perda de contagem em rajada, diagnosticada como
  "encoder ruim".
- **udev entregando `/dev/gpiomem*` ao grupo `gpio`**. O Ubuntu não faz isso por
  padrão, ao contrário do Raspberry Pi OS.

**Regra de segurança aprendida na marra:** prioridade de tempo real só com a
thread **presa num core**. Rodando `SCHED_FIFO 80` sem afinidade, com o throttling
desligado, a máquina ficou inutilizável — os spawners do `ros2_control` nunca
alcançaram o `controller_manager` e o load foi a 14. O driver hoje recusa subir
para RT se a afinidade falhar, e roda em prioridade normal com aviso. Custo baixo:
a taxa medida é a mesma com e sem RT; o RT compra latência de pior caso, não vazão.

## Calibração dos sinais

O sentido de cada roda é **medido**, não deduzido de nomenclatura. Duas medidas
independentes, feitas em 2026-09-01:

1. **Cutucada** (pulso de "frente" numa roda por vez): as quatro contaram
   negativo, com zero diafonia entre canais.
2. **Giro à mão no sentido de marcha à frente**: FL +, FR −, BL +, BR −.

A diferença entre as duas é o **espelhamento mecânico das rodas esquerdas**, que
já era tratado por `command_sign`/`feedback_sign`. Daí `encoder_sign = −1`
uniforme, que alinha o decodificador com o espaço de pulso e preserva a convenção
de junta que a odometria já usava.

Confirmação independente: o resíduo de coerência mecanum
`e = ω_FL + ω_FR − ω_BL − ω_BR` ficou em **0,027 ± 0,119 rad/s** com as quatro
rodas a 6 rad/s. Se qualquer roda estivesse com sinal invertido, esse resíduo
seria ~12 rad/s.

## Pendências conhecidas

- **Escala**: em 8 medições independentes a contagem deu 2 a 3% abaixo do
  esperado, sempre no mesmo sentido. Ou é parada antes da marca de forma
  consistente, ou a redução nominal "28:1" é fracionária. A Etapa 1 da
  calibração de odometria absorve isso no `wheels_radius`.
- **Assimetria entre lados**: com o mesmo comando, as rodas da direita rodam ~6%
  mais devagar que as da esquerda para frente. Tentar corrigir por um offset de
  pulso inferido **piorou** (foi para 24%) — a resposta real a um degrau de pulso
  não bate com a reta afim que o driver assume. Resolver isso exige varrer
  **pulsos explícitos** por roda e medir a curva real de cada placa, como
  `calibracao_odometria.md` §5 já prescreve. Nota: essa assimetria também existe
  na Pi 4, onde o robô opera bem, então não é regressão do port.
