#!/usr/bin/env python3
"""
激光遮挡过滤器节点：以雷达自身正方向（0° = 车尾）为起点，按给定角度区间遮挡/保留扫描点。

规则（顺时针）：先不挡 24.83°，再挡 38.52°，不挡 43.48°，再挡 42.36°；另一侧对称。
即遮挡区间（度）：[24.83, 63.35]、[106.83, 149.19]、[-63.35, -24.83]、[-149.19, -106.83]。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


def deg2rad(d: float) -> float:
    return math.radians(d)


class FourWheelFilter(Node):
    def __init__(self):
        super().__init__('four_wheel_filter')

        # 以雷达为正方向（0° = 车尾），顺时针：走 24.83° 后挡 38.52°，不挡 43.48°，再挡 42.36°；另一侧对称
        # 遮挡区间（弧度）[min, max]：右侧 [24.83°, 63.35°]、[106.83°, 149.19°]，左侧 [−63.35°, −24.83°]、[−149.19°, −106.83°]
        self.blocked_ranges = [
            (deg2rad(24.83), deg2rad(63.35)),
            (deg2rad(106.83), deg2rad(149.19)),
            (deg2rad(-63.35), deg2rad(-24.83)),
            (deg2rad(-149.19), deg2rad(-106.83)),
        ]
        self._first_scan_received = False

        self.declare_parameter('min_range', 0.2)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_filtered')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, sensor_qos
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, 15)

        self.get_logger().info(
            '遮挡过滤器已启动，遮挡区间(°): '
            + str([f'{math.degrees(r[0]):.1f}°~{math.degrees(r[1]):.1f}°' for r in self.blocked_ranges])
        )

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_in_blocked_range(self, angle: float) -> bool:
        """检查角度是否在任一遮挡区间内（含跨 -π/π 边界）。"""
        angle = self._normalize_angle(angle)
        for min_a, max_a in self.blocked_ranges:
            if min_a > max_a:
                if angle >= min_a or angle <= max_a:
                    return True
            else:
                if min_a <= angle <= max_a:
                    return True
        return False

    def scan_callback(self, msg: LaserScan) -> None:
        if not self._first_scan_received:
            self._first_scan_received = True
            input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
            output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
            self.get_logger().info(f'已收到首帧 {input_topic}，开始发布 {output_topic}')
        out = LaserScan()
        out.header = msg.header #<--- 记得改动！！！模拟时间/系统时间！！！
        # out.header.frame_id = msg.header.frame_id
        # out.header.stamp = self.get_clock().now().to_msg()  # 使用当前时钟（仿真时为 sim time，与 collision_monitor 一致）
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        min_range = self.get_parameter('min_range').get_parameter_value().double_value
        max_range = self.get_parameter('max_range').get_parameter_value().double_value
        has_intensities = len(msg.intensities) == len(msg.ranges)

        ranges_out = []
        intensities_out = []
        angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            invalid = r < min_range or r > max_range or self.is_in_blocked_range(angle)
            if invalid:
                ranges_out.append(float('inf'))
                intensities_out.append(0.0 if has_intensities else 0.0)
            else:
                ranges_out.append(float(r))
                intensities_out.append(msg.intensities[i] if has_intensities else 0.0)
            angle += msg.angle_increment

        out.ranges = ranges_out
        if has_intensities:
            out.intensities = intensities_out
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
