#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SensorExplore(Node):
    def __init__(self):
        super().__init__('sensor_explore')

        self.declare_parameter('linear_speed',    0.2)
        self.declare_parameter('angular_speed',   0.6)
        self.declare_parameter('safe_distance',   0.5)
        # 前方と判定する半角 [度]
        self.declare_parameter('front_half_deg',  30.0)
        # 左右センサー扇形の中心角と半幅 [度]
        self.declare_parameter('side_center_deg', 90.0)
        self.declare_parameter('side_half_deg',   40.0)

        lin  = self.get_parameter('linear_speed').value
        ang  = self.get_parameter('angular_speed').value
        self.safe_dist    = self.get_parameter('safe_distance').value
        fh   = math.radians(self.get_parameter('front_half_deg').value)
        sc   = math.radians(self.get_parameter('side_center_deg').value)
        sh   = math.radians(self.get_parameter('side_half_deg').value)

        self.lin_speed   = lin
        self.ang_speed   = ang
        # 前方扇形: [-fh, +fh]
        self.front_lo, self.front_hi = -fh, fh
        # 左扇形: [sc-sh, sc+sh]
        self.left_lo,  self.left_hi  =  sc - sh,  sc + sh
        # 右扇形: [-(sc+sh), -(sc-sh)]
        self.right_lo, self.right_hi = -(sc + sh), -(sc - sh)

        self.sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(
            f'sensor_explore 起動: safe={self.safe_dist}m '
            f'lin={self.lin_speed} ang={self.ang_speed}'
        )

    # ------------------------------------------------------------------
    def _sector_min(self, scan, lo, hi):
        """扇形 [lo, hi] 内の有効レンジの最小値を返す (有効読み取りがなければ range_max)."""
        best = scan.range_max
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))  # -π〜π に正規化
            if lo <= a <= hi and scan.range_min < r:
                best = min(best, r)
        return best

    def _sector_avg(self, scan, lo, hi):
        """扇形 [lo, hi] 内の有効レンジの平均値を返す (有効読み取りがなければ 0)."""
        vals = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if lo <= a <= hi and scan.range_min < r:
                vals.append(min(r, scan.range_max))
        return sum(vals) / len(vals) if vals else 0.0

    # ------------------------------------------------------------------
    def _on_scan(self, scan: LaserScan):
        min_front = self._sector_min(scan, self.front_lo, self.front_hi)

        twist = Twist()

        if min_front > self.safe_dist:
            # 前方が開いている → 直進
            twist.linear.x = self.lin_speed
        else:
            # 障害物あり → 左右の平均距離を比較して広い方へ回転
            avg_left  = self._sector_avg(scan, self.left_lo,  self.left_hi)
            avg_right = self._sector_avg(scan, self.right_lo, self.right_hi)

            self.get_logger().debug(
                f'obstacle: front={min_front:.2f} left={avg_left:.2f} right={avg_right:.2f}'
            )

            # 左≧右なら左回転 (正)、右が広ければ右回転 (負)
            twist.angular.z = self.ang_speed if avg_left >= avg_right else -self.ang_speed

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SensorExplore()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
