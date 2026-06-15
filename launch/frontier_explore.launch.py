import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# tf2_ros::TransformBroadcaster は常に絶対パス /tf に publish する。
# しかし tf2_ros::TransformListener は相対 tf を subscribe するため、
# namespace robot_N 内では /robot_N/tf を見てしまい /tf と不一致になる。
# remappings で 'tf' → '/tf' にして全ノードがグローバル TF ツリーを参照するよう修正。
TF_REMAP = [('tf', '/tf'), ('tf_static', '/tf_static')]


def make_nav2_nodes(robot_name: str, params_file: str) -> list:
    """robot_name ネームスペース用の Nav2 ノード群を生成する。"""
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
            parameters=[params_file],
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
                ],
            }],
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('multi_explore_mapping')
    nav2_params_r1 = os.path.join(pkg_dir, 'config', 'nav2_robot1_params.yaml')
    nav2_params_r2 = os.path.join(pkg_dir, 'config', 'nav2_robot2_params.yaml')

    nav2_nodes = (
        make_nav2_nodes('robot_1', nav2_params_r1)
        + make_nav2_nodes('robot_2', nav2_params_r2)
    )

    common_params = {
        'use_sim_time':      True,
        'global_frame':      'map',
        'map_topic':         '/map',
        'planner_frequency': 1.0,
        'progress_timeout':  30.0,
        'min_frontier_size': 0.3,      # 0.5→0.3: 開けた場所の小クラスタも有効化
        'potential_scale':   1.0,      # 3.0→1.0: 距離ペナルティを軽減
        'gain_scale':        3.0,      # 1.0→3.0: 未探索面積の報酬を強調→遠くの大フロンティアへ
        'visualize':         True,
        'blacklist_radius':  1.0,      # 到達失敗フロンティアの無効化半径 [m]
        'blacklist_clear_sec': 60.0,   # ブラックリストをクリアする周期 [s]
    }

    frontier_r1 = Node(
        package='multi_explore_mapping',
        executable='frontier_explore_node',
        name='frontier_explorer',
        namespace='robot_1',
        output='screen',
        parameters=[{**common_params, 'robot_base_frame': 'robot_1/base_footprint'}],
        remappings=[
            ('navigate_to_pose', '/robot_1/navigate_to_pose'),
            *TF_REMAP,
        ],
    )

    frontier_r2 = Node(
        package='multi_explore_mapping',
        executable='frontier_explore_node',
        name='frontier_explorer',
        namespace='robot_2',
        output='screen',
        parameters=[{**common_params, 'robot_base_frame': 'robot_2/base_footprint'}],
        remappings=[
            ('navigate_to_pose', '/robot_2/navigate_to_pose'),
            *TF_REMAP,
        ],
    )

    # Nav2 が lifecycle active になってから Frontier Explorer を起動
    delayed_frontiers = TimerAction(period=20.0, actions=[frontier_r1, frontier_r2])

    return LaunchDescription([
        *nav2_nodes,
        delayed_frontiers,
    ])
