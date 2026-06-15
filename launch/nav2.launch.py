#!/usr/bin/env python3
"""
Nav2 Multi-Robot Launch File
=============================
nav2_robot1_params.yaml / nav2_robot2_params.yaml を使って
robot_1, robot_2 それぞれの namespace で Nav2 を起動する。

起動順序:
  1. simulation_world.launch.py  (Gazebo + SLAM)
  2. map_matching.launch.py      (ICP 地図統合)
  3. nav2.launch.py              (このファイル)  ← /robot_*/navigate_to_pose が立ち上がる
  4. frontier_explore.launch.py  (フロンティア探索)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_my   = get_package_share_directory('multi_explore_mapping')

    nav2_launch = os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')

    robots = [
        ('robot_1', os.path.join(pkg_my, 'config', 'nav2_robot1_params.yaml')),
        ('robot_2', os.path.join(pkg_my, 'config', 'nav2_robot2_params.yaml')),
    ]

    nav2_nodes = []
    for ns, params_file in robots:
        nav2_nodes.append(
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
                        # slam_toolbox が地図・自己位置を担うので不要
                        'slam':          'false',
                        'map':           '',
                    }.items()
                )
            ])
        )

    # Gazebo + SLAM が安定してから起動（10秒待機）
    return LaunchDescription([
        TimerAction(period=10.0, actions=nav2_nodes)
    ])