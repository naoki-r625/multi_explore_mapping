#!/usr/bin/env python3
"""
Explore Supervisor Launch File
==============================
Launches the multi-robot exploration supervisor node.

This node monitors:
- Individual robot map updates
- Merged map integration status
- Robot activity and health
- Overall exploration progress
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the exploration supervisor
    """
    
    # ==================== Explore Supervisor Node ====================
    supervisor_node = Node(
        package='multi_explore_mapping',
        executable='explore_supervisor',
        name='explore_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_robots': 2,                    # Number of robots in system
            'loop_rate_hz': 1.0,               # Monitoring frequency (1 Hz)
            'merge_timeout_sec': 10.0,         # Timeout for robot inactivity
        }]
    )

    return LaunchDescription([
        supervisor_node,
    ])