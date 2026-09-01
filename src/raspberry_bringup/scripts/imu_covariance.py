#!/usr/bin/env python3
"""Republica a IMU com covariancias preenchidas.

POR QUE ISTO EXISTE
-------------------
O driver `wit_ros2_imu` publica `/imu/data_raw` com TODAS as covariancias em
zero (confirmado no fonte: ele monta a mensagem e nunca toca nos campos
`*_covariance`). O `robot_localization` usa a covariancia que vem na mensagem e
NAO tem parametro para sobrescrever — entao covariancia zero vira confianca
praticamente infinita naquela medida.

No nosso EKF isso e' especialmente ruim: `imu0_config` funde apenas `vyaw`
(ekf.yaml), e o giro desta IMU e' quantizado em 0,061 graus/s (LSB de 16 bits em
+-2000 graus/s). Em repouso ele reporta exatamente 0,0 em 99,3% das amostras
(medido: 1172 de 1180 em 120 s). Zero com confianca infinita, dez vezes por
segundo, e' o EKF sendo instruido a acreditar que a velocidade angular e'
exatamente nula — brigando com a odometria de roda em manobras lentas, que sao
justamente as de docagem.

Editar o driver localmente esta explicitamente vetado (`docs/raspberry_tempo_real.md`:
pacote de terceiros em ~/ros2_ws, nao editar). Entao republicamos, no mesmo
padrao que o `scan_normalizer.py` ja usa para o LiDAR.

CALIBRACAO DAS COVARIANCIAS
---------------------------
Medido em 2026-09-01 com o robo parado e os motores DESENERGIZADOS:
  bias de vyaw : +6e-06 rad/s  (+0,022 graus/min de deriva de yaw)
  variancia    : 4,1e-08 (rad/s)^2
A variancia de um quantizador uniforme e' q^2/12 = 9,5e-08, ou seja, o ruido
medido e' dominado pela quantizacao — como esperado com o robo parado.

ATENCAO: essa medida e' um PISO. Com os motores girando existe vibracao
mecanica, que nao entra numa medicao em repouso. Por isso o default aqui e'
deliberadamente mais conservador que o medido, e precisa ser refeito com o robo
em movimento (a bateria acabou antes de dar tempo). Um valor bom demais e' pior
que um valor conservador: o EKF passa a ignorar a odometria de roda.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

# Marcador do REP-145 para "este campo nao e' fornecido": o primeiro elemento
# da covariancia em -1. Usado nos campos que a IMU nao mede de forma confiavel.
NAO_FORNECIDO = -1.0


class ImuCovariance(Node):
    def __init__(self) -> None:
        super().__init__('imu_covariance')

        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data')
        # Variancia de velocidade angular, por eixo (rad/s)^2. O default e' ~600x
        # o ruido medido em repouso, para cobrir vibracao com os motores girando.
        # Recalibrar com o robo em movimento e ajustar aqui.
        self.declare_parameter('angular_velocity_variance', 2.5e-5)
        # Aceleracao linear: nao e' fundida pelo nosso EKF (imu0_config tem as
        # aceleracoes em false), mas publicar valor sensato evita que outro
        # consumidor herde o zero.
        self.declare_parameter('linear_acceleration_variance', 4.0e-2)
        # Orientacao: a WIT publica um quaternion, mas o EKF nao o funde e nao
        # temos referencia magnetica confiavel dentro do robo (motores e
        # correntes altas). -1 marca "nao fornecido" conforme a REP-145.
        self.declare_parameter('publish_orientation', False)

        self._entrada = self.get_parameter('input_topic').get_parameter_value().string_value
        self._saida = self.get_parameter('output_topic').get_parameter_value().string_value
        var_w = self.get_parameter('angular_velocity_variance').get_parameter_value().double_value
        var_a = self.get_parameter(
            'linear_acceleration_variance').get_parameter_value().double_value
        self._com_orientacao = self.get_parameter(
            'publish_orientation').get_parameter_value().bool_value

        self._cov_w = [var_w, 0.0, 0.0, 0.0, var_w, 0.0, 0.0, 0.0, var_w]
        self._cov_a = [var_a, 0.0, 0.0, 0.0, var_a, 0.0, 0.0, 0.0, var_a]

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Imu, self._saida, qos)
        self._sub = self.create_subscription(Imu, self._entrada, self._on_imu, qos)

        self._n = 0
        self._avisou_nan = False
        self.get_logger().info(
            f'{self._entrada} -> {self._saida} | var(vyaw)={var_w:.3e} (rad/s)^2, '
            f'orientacao {"publicada" if self._com_orientacao else "marcada como nao fornecida"}')

    def _on_imu(self, msg: Imu) -> None:
        saida = Imu()
        saida.header = msg.header
        saida.angular_velocity = msg.angular_velocity
        saida.linear_acceleration = msg.linear_acceleration
        saida.angular_velocity_covariance = self._cov_w
        saida.linear_acceleration_covariance = self._cov_a

        if self._com_orientacao:
            saida.orientation = msg.orientation
            saida.orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        else:
            saida.orientation_covariance = [NAO_FORNECIDO] + [0.0] * 8

        # NaN na entrada envenena o EKF em silencio: melhor descartar e avisar.
        if not all(math.isfinite(v) for v in (
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z)):
            if not self._avisou_nan:
                self.get_logger().error('IMU publicou valor nao-finito; mensagem descartada.')
                self._avisou_nan = True
            return

        self._pub.publish(saida)
        self._n += 1


def main() -> None:
    rclpy.init()
    node = ImuCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
