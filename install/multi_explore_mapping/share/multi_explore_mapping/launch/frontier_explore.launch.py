import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir     = get_package_share_directory('multi_explore_mapping')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    nav2_params_r1 = os.path.join(pkg_dir, 'config', 'nav2_robot1_params.yaml')
    nav2_params_r2 = os.path.join(pkg_dir, 'config', 'nav2_robot2_params.yaml')

    # ── Nav2 (robot_1) ────────────────────────────────────────────────
    # use_namespace=True でノードが /robot_1 ネームスペースに入る
    # cmd_vel → /robot_1/cmd_vel、フレーム名は params YAML で明示指定
    nav2_r1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'namespace':       'robot_1',
            'use_namespace':   'True',
            'params_file':     nav2_params_r1,
            'use_sim_time':    'True',
            'autostart':       'True',
            'use_composition': 'False',
        }.items()
    )

    # ── Nav2 (robot_2) ────────────────────────────────────────────────
    nav2_r2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'namespace':       'robot_2',
            'use_namespace':   'True',
            'params_file':     nav2_params_r2,
            'use_sim_time':    'True',
            'autostart':       'True',
            'use_composition': 'False',
        }.items()
    )

    common_params = {
        'use_sim_time':      True,
        'global_frame':      'map',
        'map_topic':         '/map',
        'planner_frequency': 1.0,
        'progress_timeout':  30.0,
        'min_frontier_size': 0.5,
        'potential_scale':   3.0,
        'gain_scale':        1.0,
        'visualize':         True,
    }

    # ── Frontier Explorer (robot_1) ───────────────────────────────────
    frontier_r1 = Node(
        package='multi_explore_mapping',
        executable='frontier_explore_node',
        name='frontier_explorer',
        namespace='robot_1',
        output='screen',
        parameters=[{
            **common_params,
            'robot_base_frame': 'robot_1/base_footprint',
        }],
        remappings=[
            ('navigate_to_pose', '/robot_1/navigate_to_pose'),
        ],
    )

    # ── Frontier Explorer (robot_2) ───────────────────────────────────
    frontier_r2 = Node(
        package='multi_explore_mapping',
        executable='frontier_explore_node',
        name='frontier_explorer',
        namespace='robot_2',
        output='screen',
        parameters=[{
            **common_params,
            'robot_base_frame': 'robot_2/base_footprint',
        }],
        remappings=[
            ('navigate_to_pose', '/robot_2/navigate_to_pose'),
        ],
    )

    # Nav2 が起動してから Frontier Explorer を開始（20秒待機）
    delayed_frontiers = TimerAction(
        period=20.0,
        actions=[frontier_r1, frontier_r2]
    )

    return LaunchDescription([
        nav2_r1,
        nav2_r2,
        delayed_frontiers,
    ])
