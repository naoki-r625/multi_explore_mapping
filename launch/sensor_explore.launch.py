#!/usr/bin/env python3
"""
Sensor Explore Launch File (C++ Version)
========================================
Launches sensor-based exploration nodes with anti-oscillation
escape behavior for multiple robots.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        'use_sim_time': True,

        # --- 基本動作 ---
        'linear_speed': 0.2,           # 前進速度 (m/s)
        'angular_speed': 0.6,          # 回転速度 (rad/s)
        'safe_distance': 0.5,          # 障害物安全距離 (m)
        'front_half_deg': 30.0,        # 前方検出範囲 ±30°
        'side_center_deg': 90.0,       # 側方検出中心 90°
        'side_half_deg': 40.0,         # 側方検出幅 ±40°

        # --- 脱出行動 ---
        'escape_back_speed': 0.15,     # 後退速度 (m/s)
        'escape_turn_speed': 0.8,      # 脱出回転速度 (rad/s)
        'escape_back_ticks': 15,       # 後退 tick 数 (×100ms = 1.5秒)
        'escape_turn_ticks_min': 10,   # 回転 最小 tick (×100ms = 1.0秒)
        'escape_turn_ticks_max': 30,   # 回転 最大 tick (×100ms = 3.0秒)

        # --- 振動検出 ---
        'oscillation_window': 20,      # 直近 20 tick を監視 (2秒)
        'oscillation_threshold': 6,    # 左右切替が 6回 以上 → 振動判定

        # --- 回転ホールド ---
        'turn_hold_ticks': 8,          # 回転方向を 8 tick (0.8秒) 維持
    }

    return LaunchDescription([
        Node(
            package='multi_explore_mapping',
            executable='sensor_explore_node',
            name='sensor_explore',
            namespace='robot_1',
            parameters=[common_params],
            output='screen',
        ),
        Node(
            package='multi_explore_mapping',
            executable='sensor_explore_node',
            name='sensor_explore',
            namespace='robot_2',
            parameters=[common_params],
            output='screen',
        ),
    ])