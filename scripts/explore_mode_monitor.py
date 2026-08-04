#!/usr/bin/env python3
"""
Explore Mode Monitor
====================
ロボットごとに /<robot>/map の既知セル数(-1以外)の増加量を監視し、
window_sec あたりの増加量が min_increase_cells を sustained_checks 回
連続で下回ったら、そのロボット名を含むマーカー行を stdout に出力する。

adaptive_explore.launch.py 側がこの stdout を監視し、対象ロボットの
sensor_explore_node を停止して Nav2 + frontier_explore_node を起動する
(センサー→フロンティアの片方向切り替え、ロボットごとに一度きり)。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from collections import deque

SWITCH_MARKER = 'EXPLORE_MODE_SWITCH robot={}'


class ExploreModeMonitor(Node):
    def __init__(self):
        super().__init__('explore_mode_monitor')

        self.declare_parameter('robot_names', ['robot_1', 'robot_2'])
        self.declare_parameter('window_sec', 60.0)
        self.declare_parameter('check_period_sec', 5.0)
        self.declare_parameter('min_increase_cells', 300.0)
        self.declare_parameter('warmup_sec', 30.0)
        self.declare_parameter('sustained_checks', 3)

        self.robots = list(self.get_parameter('robot_names').value)
        self.window_sec = self.get_parameter('window_sec').value
        self.min_increase = self.get_parameter('min_increase_cells').value
        self.warmup_sec = self.get_parameter('warmup_sec').value
        self.sustained_checks = self.get_parameter('sustained_checks').value
        check_period = self.get_parameter('check_period_sec').value

        self._latest_map = {r: None for r in self.robots}
        self._history = {r: deque() for r in self.robots}  # [(t, known_count), ...]
        self._low_streak = {r: 0 for r in self.robots}
        self._switched = {r: False for r in self.robots}
        self._start_time = self._now()

        for r in self.robots:
            self.create_subscription(
                OccupancyGrid, f'/{r}/map',
                lambda msg, r=r: self._on_map(r, msg),
                qos_profile_sensor_data,
            )

        self.create_timer(check_period, self._check)
        self.get_logger().info(
            f'explore_mode_monitor: watching {self.robots}, '
            f'window={self.window_sec}s threshold={self.min_increase}cells '
            f'warmup={self.warmup_sec}s sustained={self.sustained_checks}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, robot: str, msg: OccupancyGrid):
        self._latest_map[robot] = msg

    def _check(self):
        now = self._now()
        if now - self._start_time < self.warmup_sec:
            return

        for r in self.robots:
            if self._switched[r]:
                continue
            msg = self._latest_map[r]
            if msg is None:
                continue

            known = sum(1 for c in msg.data if c != -1)
            hist = self._history[r]
            hist.append((now, known))
            cutoff = now - self.window_sec
            while len(hist) > 1 and hist[0][0] < cutoff:
                hist.popleft()

            oldest_t, oldest_known = hist[0]
            if now - oldest_t < self.window_sec * 0.8:
                continue  # 十分な履歴が溜まるまで判定しない

            increase = known - oldest_known
            if increase < self.min_increase:
                self._low_streak[r] += 1
            else:
                self._low_streak[r] = 0

            self.get_logger().debug(
                f'{r}: known={known} increase={increase:.0f}/{self.window_sec:.0f}s '
                f'streak={self._low_streak[r]}/{self.sustained_checks}'
            )

            if self._low_streak[r] >= self.sustained_checks:
                self._switched[r] = True
                self.get_logger().warn(
                    f'{r}: area growth below threshold -> switching to frontier mode'
                )
                print(SWITCH_MARKER.format(r), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreModeMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
