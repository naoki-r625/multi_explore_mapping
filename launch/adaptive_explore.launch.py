#!/usr/bin/env python3
"""
Adaptive Explore Launch File
============================
ロボットごとにセンサーベース探査(sensor_explore_node)で開始し、
explore_mode_monitor が各ロボットの /<robot>/map の増加量を監視する。
増加量が閾値を下回ったロボットだけを Nav2 + frontier_explore_node に
切り替える(センサー→フロンティアの片方向切り替え、ロボットごとに独立)。

単体で使いたい場合は sensor_explore.launch.py / frontier_explore.launch.py を
このファイルとは別に個別起動できる(このファイルはそれらに依存しない)。
"""
import os
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessIO
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

TF_REMAP = [('tf', '/tf'), ('tf_static', '/tf_static')]

SENSOR_PARAMS = {
    'use_sim_time':          True,
    'linear_speed':          0.2,
    'angular_speed':         0.6,
    'safe_distance':         0.5,
    'vfh_threshold':         1.5,
    'valley_min_deg':       30.0,
    'robot_radius':         0.13,
    'emergency_dist':        0.22,
    'front_cone_deg':       30.0,
    'min_gap_width':         0.4,
    'dup_radius':            1.5,
    'dup_time':            600.0,
    'frontier_ttl':        300.0,
    'odom_frame': 'map',
    'use_voronoi_partition': True,
}

FRONTIER_PARAMS = {
    'use_sim_time':      True,
    'global_frame':      'map',
    'planner_frequency': 2.0,
    'progress_timeout':  90.0,
    'min_frontier_size': 0.3,
    'potential_scale':   0.5,
    'gain_scale':        3.0,
    'visualize':         True,
    'blacklist_radius':  1.0,
    'blacklist_clear_sec': 60.0,
    'use_voronoi_partition': True,
}

MONITOR_PARAMS = {
    'use_sim_time':       True,
    'window_sec':         60.0,
    'check_period_sec':    5.0,
    'min_increase_cells': 300.0,  # 環境・地図解像度に応じて要調整
    'warmup_sec':         30.0,
    'sustained_checks':    3,
}

# voronoi_partition_node: 統合地図上でロボットごとの担当領域(Voronoi)を計算し、
# /<robot>/voronoi_mask として配信する。センサーベース/フロンティアベース
# どちらのフェーズでも、探査中のロボット位置を生成点として継続的に再計算する
# ため、モード切替のタイミングとは独立に常時起動しておく(詳細は
# docs/voronoi_partition.md)。
VORONOI_PARAMS = {
    'use_sim_time':          True,
    'global_frame':          'map',
    'base_frame_suffix':     'base_footprint',
    'map_topic':              '/map',   # icp_map_matching_node が配信する統合地図
    # icp_map_matching_node の overlap_filter_margin の既定値(2.0m)と揃えて
    # いる。担当領域の境界に、常にICP対応点が取れるだけの重複帯を残すため。
    # map_matching.launch.py で overlap_filter_margin を変更した場合はここも合わせる。
    'buffer_width_m':          2.0,
    'recompute_period_sec':    5.0,
    'downsample_factor':         2,
    'obstacle_threshold':       50,
    # 生存者バイアス: 前回サイクルの担当ロボットが、この差[m]より縮まらない
    # 限りは他ロボットに明け渡さない。ロボット位置や新発見の通路次第で毎回の
    # 最近傍計算だけだと担当領域が左右で入れ替わってしまうことがあるため、
    # ちらつき・往復を抑える。0にすると旧来のヒステリシスなし挙動に戻る。
    'hysteresis_margin_m':     1.5,
}

# Nav2 用パラメータファイルは robot_1 / robot_2 分しか用意されていないため、
# このファイルもその2台を対象とする(増やす場合は config/ に nav2_robotN_params.yaml を追加)。
ROBOTS = [
    ('robot_1', 'nav2_robot1_params.yaml'),
    ('robot_2', 'nav2_robot2_params.yaml'),
]


def make_nav2_nodes(robot_name: str, params_file: str) -> list:
    """robot_name ネームスペース用の Nav2 ノード群を生成する。"""
    # $(find-pkg-share ...) はNode(parameters=[path])のように素のパス文字列
    # で渡したYAML内では展開されない(launch_rosのParameterFileがデフォルト
    # allow_substs=Falseで包むため)。実パスはここでPythonで解決してから
    # bt_navigatorにだけ追加パラメータとして渡す。
    bt_xml_path = os.path.join(
        get_package_share_directory('multi_explore_mapping'),
        'config', 'nav2_bt_fast_fail.xml')

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_name,
            output='screen',
            parameters=[params_file],
            remappings=TF_REMAP,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_name,
            output='screen',
            parameters=[params_file],
            remappings=TF_REMAP,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_name,
            output='screen',
            parameters=[params_file, {'default_nav_to_pose_bt_xml': bt_xml_path}],
            remappings=TF_REMAP,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_name,
            output='screen',
            parameters=[params_file],
            remappings=TF_REMAP,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server',
                    'global_costmap/global_costmap',
                    'local_costmap/local_costmap',
                ],
            }],
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_explore_mapping')

    sensor_nodes = {}
    nav2_node_groups = {}
    frontier_nodes = {}
    launch_actions = []

    for robot, nav2_params_name in ROBOTS:
        sensor_node = Node(
            package='multi_explore_mapping',
            executable='sensor_explore_node',
            name='sensor_explore',
            namespace=robot,
            parameters=[SENSOR_PARAMS],
            output='screen',
        )
        sensor_nodes[robot] = sensor_node
        launch_actions.append(sensor_node)

        nav2_params_file = os.path.join(pkg_dir, 'config', nav2_params_name)
        nav2_node_groups[robot] = make_nav2_nodes(robot, nav2_params_file)

        frontier_nodes[robot] = Node(
            package='multi_explore_mapping',
            executable='frontier_explore_node',
            name='frontier_explorer',
            namespace=robot,
            output='screen',
            parameters=[{**FRONTIER_PARAMS,
                         'robot_base_frame': f'{robot}/base_footprint',
                         'map_topic': f'/{robot}/global_costmap/costmap'}],
            remappings=[
                ('navigate_to_pose', f'/{robot}/navigate_to_pose'),
                *TF_REMAP,
            ],
        )

    monitor_node = Node(
        package='multi_explore_mapping',
        executable='explore_mode_monitor.py',
        name='explore_mode_monitor',
        output='screen',
        parameters=[{**MONITOR_PARAMS, 'robot_names': [r for r, _ in ROBOTS]}],
    )
    launch_actions.append(monitor_node)

    # ロボットの探査モード(センサー/フロンティア)に関係なく常時起動。
    # 生成点はTF経由の現在位置なので、どちらのモードのロボットでも
    # 自動的に担当領域の計算に参加する。
    voronoi_node = Node(
        package='multi_explore_mapping',
        executable='voronoi_partition_node',
        name='voronoi_partition',
        output='screen',
        parameters=[{**VORONOI_PARAMS, 'robot_names': [r for r, _ in ROBOTS]}],
    )
    launch_actions.append(voronoi_node)

    already_switched = set()

    def handle_monitor_output(event):
        text = event.text.decode(errors='replace')
        new_actions = []
        for robot in sensor_nodes:
            if robot in already_switched:
                continue
            if f'EXPLORE_MODE_SWITCH robot={robot}' not in text:
                continue
            already_switched.add(robot)
            new_actions.append(
                EmitEvent(event=ShutdownProcess(
                    process_matcher=matches_action(sensor_nodes[robot])
                ))
            )
            # sensor_explore_node の cmd_vel 停止が反映されるまで少し待ってから
            # Nav2 + frontier を起動する(直後だと停止前の速度指令と競合し得る)。
            new_actions.append(
                TimerAction(period=3.0, actions=[
                    *nav2_node_groups[robot],
                    frontier_nodes[robot],
                ])
            )
        return new_actions or None

    launch_actions.append(
        RegisterEventHandler(
            OnProcessIO(
                target_action=monitor_node,
                on_stdout=handle_monitor_output,
                on_stderr=handle_monitor_output,
            )
        )
    )

    return LaunchDescription(launch_actions)
