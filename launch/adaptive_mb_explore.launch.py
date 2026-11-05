#!/usr/bin/env python3
"""
Adaptive Explore Launch File (Nav2-driven sensor phase)
=========================================================
adaptive_explore.launch.py と同じハイブリッド探査(センサーベース→
フロンティアベースへの片方向切り替え)だが、センサーベースのフェーズも
VFHでcmd_velを直接叩くsensor_explore_nodeではなく、Nav2(navigate_to_pose)
に移動を任せるsensor_mb_explore_nodeを使う。

adaptive_explore.launch.pyとの構造上の違い:
  - Nav2は「切り替わってから」ではなく、最初から両ロボット分起動する。
    sensor_mb_explore_node自体がNav2のアクションクライアントなので、
    フェーズ1の時点でNav2が立っている必要があるため
    (adaptive_explore.launch.pyはVFHで直接cmd_velを叩くので、Nav2は
    切り替わるまで不要だった)。
  - モード切替時にNav2の起動は不要になった分、処理は単純になっている:
    該当ロボットのsensor_mb_explore_nodeを止めてfrontier_explore_nodeを
    起動するだけ(Nav2のcontroller_server等はそのまま動き続ける)。

単体で使いたい場合は sensor_mb_explore.launch.py / frontier_explore.launch.py
をこのファイルとは別に個別起動できる(このファイルはそれらに依存しない)。
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

# センサーベース探査(ギャップ検出)のパラメータ。sensor_mb_explore.launch.py
# と同じ(linear_speed/angular_speedはcmd_vel直接制御用だったのでここでは不要)。
SENSOR_MB_PARAMS = {
    'use_sim_time':          True,
    'safe_distance':         0.5,
    'vfh_threshold':         1.5,
    'valley_min_deg':       30.0,
    'robot_radius':         0.13,
    'emergency_dist':        0.22,
    'front_cone_deg':       30.0,
    # 分岐点検出(docs/gap_extraction_spec.md)。他はコード側のデフォルト
    # (golden データで検証済みの値)のままでよい。
    'gap_min_width':         0.6,
    'dup_radius':            1.5,
    'dup_time':            600.0,
    'frontier_ttl':        300.0,
    'odom_frame':          'map',
    'global_frame':        'map',
    'use_voronoi_partition': True,
    'filter_explored_frontiers': True,
    'merged_map_topic':    '/map',
    'explored_obstacle_threshold': 50,
    'progress_timeout':     90.0,
    'blacklist_radius':      1.0,
    'blacklist_clear_sec':  60.0,
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
    'buffer_width_m':          2.0,
    'recompute_period_sec':    5.0,
    'downsample_factor':         2,
    'obstacle_threshold':       50,
    'hysteresis_margin_m':     3.0,
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

    sensor_mb_nodes = {}
    frontier_nodes = {}
    nav2_actions = []
    launch_actions = []

    for i, (robot, nav2_params_name) in enumerate(ROBOTS):
        nav2_params_file = os.path.join(pkg_dir, 'config', nav2_params_name)
        nav2_nodes = make_nav2_nodes(robot, nav2_params_file)
        # robot_2以降のNav2は10秒ずつ遅らせて起動: 同時起動するとbt_navigatorの
        # 初期化が競合してどちらかが失敗するため(frontier_explore.launch.py等と同じ)。
        if i == 0:
            nav2_actions.extend(nav2_nodes)
        else:
            nav2_actions.append(TimerAction(period=10.0 * i, actions=nav2_nodes))

        peer_namespaces = [r for r, _ in ROBOTS if r != robot]

        sensor_mb_node = Node(
            package='multi_explore_mapping',
            executable='sensor_mb_explore_node',
            name='sensor_mb_explore',
            namespace=robot,
            output='screen',
            parameters=[SENSOR_MB_PARAMS],
            remappings=[
                ('navigate_to_pose', f'/{robot}/navigate_to_pose'),
                ('scan', 'scan_filtered'),
                *TF_REMAP,
            ],
        )
        sensor_mb_nodes[robot] = sensor_mb_node

        frontier_nodes[robot] = Node(
            package='multi_explore_mapping',
            executable='frontier_explore_node',
            name='frontier_explorer',
            namespace=robot,
            output='screen',
            parameters=[{**FRONTIER_PARAMS,
                         'robot_base_frame': f'{robot}/base_footprint',
                         'map_topic': f'/{robot}/global_costmap/costmap',
                         'peer_namespaces': peer_namespaces}],
            remappings=[
                ('navigate_to_pose', f'/{robot}/navigate_to_pose'),
                *TF_REMAP,
            ],
        )

    launch_actions.extend(nav2_actions)

    # Nav2のライフサイクルが安定してからセンサーベース探査(フェーズ1)を開始する。
    # sensor_mb_explore.launch.py と同じ20秒遅延。
    launch_actions.append(
        TimerAction(period=20.0, actions=list(sensor_mb_nodes.values()))
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
        for robot in sensor_mb_nodes:
            if robot in already_switched:
                continue
            if f'EXPLORE_MODE_SWITCH robot={robot}' not in text:
                continue
            already_switched.add(robot)
            new_actions.append(
                EmitEvent(event=ShutdownProcess(
                    process_matcher=matches_action(sensor_mb_nodes[robot])
                ))
            )
            # Nav2は既に起動済みなので、ここで起動するのはfrontier_explore_node
            # だけでよい。sensor_mb_explore_node停止直後に送信済みのゴールが
            # Nav2側に残っていても、frontier_explore_nodeの新しいゴールが
            # そのまま上書き(preempt)するので実害はないが、念のため少し待つ。
            new_actions.append(
                TimerAction(period=1.0, actions=[frontier_nodes[robot]])
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
