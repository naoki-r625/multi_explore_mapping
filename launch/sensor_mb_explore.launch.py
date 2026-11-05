#!/usr/bin/env python3
"""
Sensor Move-Base Explore Launch File
=====================================
センサーベース探査(LiDARのギャップ検出によるフロンティア選択)だが、
移動はVFHで自前のcmd_velを出すのではなくNav2(navigate_to_pose)に任せる。
frontier_explore.launch.py と全く同じNav2ノード構成(手動構築・TF_REMAP・
default_nav_to_pose_bt_xmlのPython側注入)を使う。局所的な障害物回避は
Nav2のcontroller_server + costmapが行う(sensor_explore_node のような
VFHによるcmd_vel直接制御はしない)。

旧 sensor_mb_explore.launch.py は nav2_bringup の navigation_launch.py を
そのままincludeしていたが、このパッケージの他のNav2連携(TFリマップ、
BT XMLパスの注入方式)と作法が異なり、かつ当時の sensor_mb_explore_node は
実際にはNav2アクションクライアントを使わずVFHでcmd_velを直接叩く
sensor_explore_node のコピーのままだった(両方が同じcmd_velを取り合って
壊れていたと考えられる)。両方を frontier_explore.launch.py 相当に作り直した。

起動順序:
  1. simulation_world.launch.py   (Gazebo + SLAM)
  2. map_matching.launch.py       (ICP地図統合。Voronoi分割がこの /map を参照する)
  3. sensor_mb_explore.launch.py  (このファイル)
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

TF_REMAP = [('tf', '/tf'), ('tf_static', '/tf_static')]


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
    nav2_params_r1 = os.path.join(pkg_dir, 'config', 'nav2_robot1_params.yaml')
    nav2_params_r2 = os.path.join(pkg_dir, 'config', 'nav2_robot2_params.yaml')

    nav2_nodes_r1 = make_nav2_nodes('robot_1', nav2_params_r1)
    # robot_2のNav2は10秒遅延起動: 同時起動するとbt_navigatorの初期化が競合してどちらかが失敗するため
    nav2_nodes_r2 = TimerAction(period=10.0, actions=make_nav2_nodes('robot_2', nav2_params_r2))

    # センサーベース探査(ギャップ検出)のパラメータ。sensor_explore_node /
    # adaptive_explore.launch.py の SENSOR_PARAMS と揃えてある
    # (linear_speed/angular_speedはcmd_vel直接制御用だったのでここでは不要)。
    common_params = {
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

    robot_names = ['robot_1', 'robot_2']

    # voronoi_partition_node: 統合地図上でロボットごとの担当領域(Voronoi)を計算し、
    # /<robot>/voronoi_mask として配信する。frontier_explore.launch.py と同じ設定
    # (詳細は docs/voronoi_partition.md)。
    voronoi_params = {
        'use_sim_time':          True,
        'global_frame':          'map',
        'base_frame_suffix':     'base_footprint',
        'map_topic':              '/map',   # icp_map_matching_node が配信する統合地図
        'buffer_width_m':          1.0,
        'recompute_period_sec':    5.0,
        'downsample_factor':         2,
        'obstacle_threshold':       50,
        'hysteresis_margin_m':     3.0,
        'robot_names':          robot_names,
    }

    voronoi_node = Node(
        package='multi_explore_mapping',
        executable='voronoi_partition_node',
        name='voronoi_partition',
        output='screen',
        parameters=[voronoi_params],
    )

    sensor_mb_r1 = Node(
        package='multi_explore_mapping',
        executable='sensor_mb_explore_node',
        name='sensor_mb_explore',
        namespace='robot_1',
        output='screen',
        parameters=[common_params],
        remappings=[
            ('navigate_to_pose', '/robot_1/navigate_to_pose'),
            ('scan', 'scan_filtered'),
            *TF_REMAP,
        ],
    )

    sensor_mb_r2 = Node(
        package='multi_explore_mapping',
        executable='sensor_mb_explore_node',
        name='sensor_mb_explore',
        namespace='robot_2',
        output='screen',
        parameters=[common_params],
        remappings=[
            ('navigate_to_pose', '/robot_2/navigate_to_pose'),
            ('scan', 'scan_filtered'),
            *TF_REMAP,
        ],
    )

    # SLAMが安定してからNav2 + 探査を開始する
    delayed_explorers = TimerAction(period=20.0, actions=[sensor_mb_r1, sensor_mb_r2])

    return LaunchDescription([
        *nav2_nodes_r1,
        nav2_nodes_r2,
        voronoi_node,
        delayed_explorers,
    ])
