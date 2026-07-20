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
        'use_sim_time':          True,
        # 基本動作
        'linear_speed':          0.2,   # 前進速度 [m/s]
        'angular_speed':         0.6,   # 回転速度 [rad/s]
        'safe_distance':         0.5,   # 障害物安全距離 [m]
        # VFH 障害物回避 (論文A 3.1節)
        'vfh_threshold':         1.5,   # セクタ密度の閾値
        'valley_min_deg':       30.0,   # 通過可能な谷の最小幅 [deg]
        'robot_radius':         0.13,  # TurtleBot3 Burger 半幅 [m] (横幅178mm)
        'emergency_dist':        0.22,   # ロボット端からの緊急停止クリアランス [m]
        'front_cone_deg':       30.0,   # 前方ブロック判定の半角 [deg] (大きくすると斜め壁も検知)
        # 分岐点検出 (論文A 3.2節)
        'min_gap_width':         0.4,   # ギャップ最小幅 [m] (未満は分岐点無視)
        # 重複探査防止 (論文A 3.3節 + 3.5.1節)
        'dup_radius':            1.5,   # 重複判定半径 [m]
        'dup_time':            200.0,   # 重複判定時間窓 [s]
        'odom_frame': 'map',   # SLAM が map→odom TF を配信している場合
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