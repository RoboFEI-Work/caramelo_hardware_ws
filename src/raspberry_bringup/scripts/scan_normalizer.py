#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    """Republish LaserScan messages on a fixed angular grid."""

    def __init__(self) -> None:
        super().__init__('scan_normalizer')

        self.declare_parameter('input_scan_topic', '/scan_raw')
        self.declare_parameter('output_scan_topic', '/scan')
        self.declare_parameter('angle_min', -math.pi / 2.0)
        self.declare_parameter('angle_max', math.pi / 2.0)
        self.declare_parameter('target_count', 0)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('range_min', 0.0)
        self.declare_parameter('fill_with_inf', True)
        self.declare_parameter('keep_intensities', True)

        self._angle_min = float(self.get_parameter('angle_min').value)
        self._angle_max = float(self.get_parameter('angle_max').value)
        self._angle_span = self._angle_max - self._angle_min
        self._target_count = int(self.get_parameter('target_count').value)
        self._range_max_override = float(self.get_parameter('range_max').value)
        self._range_min_override = float(self.get_parameter('range_min').value)
        self._fill_with_inf = bool(self.get_parameter('fill_with_inf').value)
        self._keep_intensities = bool(self.get_parameter('keep_intensities').value)

        self._output_angle_increment: Optional[float] = None
        self._reported_resize = False

        if self._angle_span <= 0.0 or self._angle_span > 2.0 * math.pi:
            raise ValueError('angle_max must be greater than angle_min and span must be <= 2*pi')

        input_topic = str(self.get_parameter('input_scan_topic').value)
        output_topic = str(self.get_parameter('output_scan_topic').value)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(LaserScan, output_topic, output_qos)
        self._subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            input_qos,
        )

        self.get_logger().info(
            'Normalizing LaserScan from '
            f'{input_topic} to {output_topic}; '
            f'crop=[{math.degrees(self._angle_min):.1f}, '
            f'{math.degrees(self._angle_max):.1f}] deg'
        )

    def _initialize_geometry(self, scan: LaserScan) -> bool:
        if self._output_angle_increment is not None:
            return True

        if self._target_count <= 0:
            input_increment = abs(float(scan.angle_increment))
            if input_increment > 0.0:
                self._target_count = int(round(self._angle_span / input_increment)) + 1
            else:
                self._target_count = len(scan.ranges)

        if self._target_count < 2:
            self.get_logger().error('Cannot normalize scan: target_count must be >= 2')
            return False

        self._output_angle_increment = (
            self._angle_span / float(self._target_count - 1)
        )

        self.get_logger().info(
            'Fixed scan geometry: '
            f'count={self._target_count}, '
            f'angle_min={self._angle_min:.6f}, '
            f'angle_max={self._angle_max:.6f}, '
            f'angle_increment={self._output_angle_increment:.9f}'
        )
        return True

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _positive_angle_delta(angle: float, reference: float) -> float:
        return (angle - reference) % (2.0 * math.pi)

    def _scan_callback(self, scan: LaserScan) -> None:
        if not self._initialize_geometry(scan):
            return

        assert self._output_angle_increment is not None

        output = LaserScan()
        output.header = scan.header
        output.angle_min = self._angle_min
        output.angle_max = self._angle_max
        output.angle_increment = self._output_angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = (
            self._range_min_override
            if self._range_min_override > 0.0
            else scan.range_min
        )
        output.range_max = (
            self._range_max_override
            if self._range_max_override > 0.0
            else scan.range_max
        )

        fill_value = math.inf if self._fill_with_inf else 0.0
        ranges = [fill_value] * self._target_count
        intensities = (
            [0.0] * self._target_count
            if self._keep_intensities and scan.intensities
            else []
        )

        valid_indices = set()
        for index, value in enumerate(scan.ranges):
            angle = self._normalize_angle(
                float(scan.angle_min) + float(index) * float(scan.angle_increment)
            )
            relative_angle = self._positive_angle_delta(angle, self._angle_min)
            if relative_angle > self._angle_span:
                continue

            target_index = int(round(relative_angle / self._output_angle_increment))
            if target_index < 0 or target_index >= self._target_count:
                continue

            if not math.isfinite(value) or value <= 0.0:
                continue

            current = ranges[target_index]
            if target_index not in valid_indices or not math.isfinite(current) or value < current:
                ranges[target_index] = float(value)
                valid_indices.add(target_index)
                if intensities and index < len(scan.intensities):
                    intensities[target_index] = float(scan.intensities[index])

        output.ranges = ranges
        output.intensities = intensities

        if len(scan.ranges) != self._target_count and not self._reported_resize:
            self.get_logger().warn(
                f'Resampling LaserScan: input count {len(scan.ranges)} -> '
                f'output count {self._target_count}. Further resize warnings are suppressed.'
            )
            self._reported_resize = True

        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNormalizer()
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
