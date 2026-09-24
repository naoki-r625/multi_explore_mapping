#!/usr/bin/env python3
"""
Sensor Move-Base Explore Launch File
=====================================
Launches Nav2 (with each robot's own SLAM map) and sensor_mb_explore_node
for each robot. Obstacle avoidance is handled by Nav2; frontier detection
uses the same gap-based algorithm as sensor_explore_node.

Launch order:
  1. simulation_world.launch.py  (Gazebo + SLAM)
  2. sensor_mb_explore.launch.py  (this file — Nav2 + sensor_mb_explore_node)

Note: Do NOT run this together with nav2.launch.py or sensor_explore.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_my   = get_package_share_directory('multi_explore_mapping')

    nav2_launch = os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')

    common_params = {
        'use_sim_time':       True,
        'safe_distance':      0.5,
        'robot_radius':       0.13,
        'dup_radius':         1.5,
        'dup_time':         600.0,
        'frontier_ttl':     300.0,
        'min_gap_width':      0.4,
        'odom_frame':       'map',
        'global_frame':     'map',
        'progress_timeout':  60.0,
        'goal_tolerance':     0.5,
        'blacklist_radius':   1.0,
    }

    robots = [
        ('robot_1', os.path.join(pkg_my, 'config', 'nav2_robot1_ownmap_params.yaml')),
        ('robot_2', os.path.join(pkg_my, 'config', 'nav2_robot2_ownmap_params.yaml')),
    ]

    nodes = []
    for ns, params_file in robots:
        nodes.append(
            GroupAction([
                PushRosNamespace(ns),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'use_sim_time':  'true',
                        'namespace':     ns,
                        'use_namespace': 'true',
                        'params_file':   params_file,
                        'autostart':     'true',
                        'slam':          'false',
                        'map':           '',
                    }.items()
                ),
            ])
        )
        nodes.append(
            Node(
                package='multi_explore_mapping',
                executable='sensor_mb_explore_node',
                name='sensor_mb_explore',
                namespace=ns,
                parameters=[common_params],
                remappings=[('scan', 'scan_filtered')],
                output='screen',
            )
        )

    # Wait for SLAM to stabilize before starting Nav2 + exploration
    return LaunchDescription([
        TimerAction(period=12.0, actions=nodes)
    ])
