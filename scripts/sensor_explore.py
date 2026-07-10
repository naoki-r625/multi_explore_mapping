#!/usr/bin/env python3
"""
Sensor-based autonomous exploration.

Algorithm (based on Paper A: Makino et al. 2019):
  - Forward when front is clear, steering toward nearest unexplored gap
  - Obstacle avoidance via VFH (Vector Field Histogram, Sec.3.1)
  - Frontier targets from adjacent-beam distance jumps (Sec.3.2 adapted for 2D LiDAR)
  - Duplicate exploration prevention via odometry history + radius/time filter (Sec.3.3)
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SensorExplore(Node):
    _N = 36  # VFH sectors (10° each)

    def __init__(self):
        super().__init__('sensor_explore')

        self.declare_parameter('linear_speed',    0.2)
        self.declare_parameter('angular_speed',   0.6)
        self.declare_parameter('safe_distance',   0.5)
        # VFH
        self.declare_parameter('vfh_threshold',   1.5)   # obstacle density threshold per sector
        self.declare_parameter('valley_min_deg',  30.0)  # minimum navigable valley width [deg]
        # Duplicate detection (Paper A Sec.3.3 + Sec.3.5.1)
        self.declare_parameter('dup_radius',      1.5)   # duplicate check radius [m]
        self.declare_parameter('dup_time',        200.0) # duplicate time window [s]

        self.lin   = self.get_parameter('linear_speed').value
        self.ang   = self.get_parameter('angular_speed').value
        self.safe  = self.get_parameter('safe_distance').value
        self.vfh_t = self.get_parameter('vfh_threshold').value
        self.v_min = math.radians(self.get_parameter('valley_min_deg').value)
        self.dup_r = self.get_parameter('dup_radius').value
        self.dup_t = self.get_parameter('dup_time').value

        self._pose    = None   # (x, y, yaw) in odom frame
        self._history = []     # [(x, y, t_sec), ...]

        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(Odometry,  'odom', self._on_odom, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(
            f'sensor_explore (VFH+gap+dup): '
            f'safe={self.safe}m lin={self.lin} ang={self.ang}'
        )

    # ------------------------------------------------------------------
    # Odometry: track pose and history

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        t = self.get_clock().now().nanoseconds * 1e-9
        self._pose = (x, y, yaw)
        self._history.append((x, y, t))
        # Retain last 10 minutes
        cutoff = t - 600.0
        self._history = [(xi, yi, ti) for xi, yi, ti in self._history if ti > cutoff]

    # ------------------------------------------------------------------
    # VFH: polar obstacle density histogram (Sec.3.1)

    def _build_vfh(self, scan):
        """Build VFH polar histogram. Returns list of N density values."""
        hist = [0.0] * self._N
        step = 2.0 * math.pi / self._N
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r >= scan.range_max * 0.99:
                continue
            a = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))   # normalise to [-π, π]
            s = int((a + math.pi) / step) % self._N
            # Weight: higher for closer obstacles (0 at 2*safe distance)
            w = max(0.0, (self.safe * 2.0 - r) / (self.safe * 2.0))
            hist[s] += w
        return hist

    def _vfh_valleys(self, hist):
        """Return (center_angle, width_rad) for each navigable valley."""
        step = 2.0 * math.pi / self._N
        min_sec = max(1, int(self.v_min / step))
        valleys = []

        # Double the array to handle circular wrap-around
        h2 = hist + hist
        run_start = None
        for i, v in enumerate(h2[:2 * self._N]):
            if v <= self.vfh_t:
                if run_start is None:
                    run_start = i
            else:
                if run_start is not None:
                    length = i - run_start
                    if length >= min_sec:
                        mid_sec = (run_start + i - 1) // 2
                        angle = (mid_sec % self._N) * step - math.pi
                        angle = math.atan2(math.sin(angle), math.cos(angle))
                        valleys.append((angle, length * step))
                    run_start = None
        if run_start is not None:
            length = 2 * self._N - run_start
            if length >= min_sec:
                mid_sec = (run_start + 2 * self._N - 1) // 2
                angle = (mid_sec % self._N) * step - math.pi
                angle = math.atan2(math.sin(angle), math.cos(angle))
                valleys.append((angle, length * step))

        return valleys

    def _front_blocked(self, hist):
        """True if any VFH sector within ±15° of forward exceeds threshold."""
        step = 2.0 * math.pi / self._N
        front_s = self._N // 2   # sector index for angle 0 (forward)
        half = max(1, int(math.radians(15.0) / step))
        return any(
            hist[(front_s + d) % self._N] > self.vfh_t
            for d in range(-half, half + 1)
        )

    # ------------------------------------------------------------------
    # Gap / frontier detection (Sec.3.2 adapted for 2D LiDAR)

    def _find_gap_targets(self, scan):
        """
        Detect frontier edges: transitions between an obstacle beam and an open
        (range_max) beam in adjacent readings.  Returns list of (angle, dist)
        sorted closest-to-forward first, excluding angles behind the robot.
        """
        ranges = scan.ranges
        n = len(ranges)
        targets = []

        for i in range(n - 1):
            r1 = ranges[i]
            r2 = ranges[i + 1]
            hit1 = math.isfinite(r1) and scan.range_min < r1 < scan.range_max * 0.97
            hit2 = math.isfinite(r2) and scan.range_min < r2 < scan.range_max * 0.97

            if hit1 == hit2:
                continue  # no transition

            a = scan.angle_min + (i + 0.5) * scan.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))

            dist = r1 if hit1 else r2

            # Exclude directions behind the robot (|a| > 135°)
            if abs(a) < math.radians(135.0):
                targets.append((a, dist))

        targets.sort(key=lambda x: abs(x[0]))
        return targets

    # ------------------------------------------------------------------
    # Duplicate detection (Sec.3.3 + Sec.3.5.1 time filter)

    def _is_duplicate(self, angle, dist):
        """
        Returns True if the target position (odom frame projection)
        was visited within dup_radius[m] AND within dup_time[s].
        """
        if self._pose is None or not self._history:
            return False

        cx, cy, yaw = self._pose
        world_angle = yaw + angle
        proj = min(dist, self.dup_r * 1.5)
        tx = cx + proj * math.cos(world_angle)
        ty = cy + proj * math.sin(world_angle)

        t_now = self.get_clock().now().nanoseconds * 1e-9
        for xi, yi, ti in self._history:
            if (math.hypot(tx - xi, ty - yi) < self.dup_r
                    and (t_now - ti) < self.dup_t):
                return True
        return False

    # ------------------------------------------------------------------
    # Main scan callback

    def _on_scan(self, scan: LaserScan):
        hist = self._build_vfh(scan)
        twist = Twist()

        if not self._front_blocked(hist):
            # Forward is clear: go straight, steer gently toward best gap
            targets = self._find_gap_targets(scan)
            target_angle = 0.0
            for a, d in targets:
                if not self._is_duplicate(a, d):
                    target_angle = a
                    break

            twist.linear.x = self.lin
            if abs(target_angle) > 0.15:
                twist.angular.z = self.ang * 0.4 * math.copysign(1.0, target_angle)

        else:
            # Forward blocked: rotate toward best VFH valley
            valleys = self._vfh_valleys(hist)
            if valleys:
                best_angle, _ = min(valleys, key=lambda v: abs(v[0]))
                twist.angular.z = self.ang * math.copysign(1.0, best_angle)
            else:
                # No valley: rotate toward less-dense half
                front_s = self._N // 2
                left  = sum(hist[front_s: front_s + self._N // 4])
                right = sum(hist[max(0, front_s - self._N // 4): front_s])
                twist.angular.z = self.ang if left <= right else -self.ang

            self.get_logger().debug(
                f'blocked: valleys={len(valleys)} steer={twist.angular.z:.2f}'
            )

        self._pub.publish(twist)


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
