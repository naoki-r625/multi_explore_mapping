import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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
        # 3/16修正: global/local_costmap もライフサイクルノードとして起動されるため、マネージャーに明記
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

    # 共通パラメータ設定
    common_params = {
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
    }

    # 【重要】トピックを共通の統合マップ(/map)をインフレーションさせた各Nav2グローバルコストマップに変更
    frontier_r1 = Node(
        package='multi_explore_mapping',
        executable='frontier_explore_node',
        name='frontier_explorer',
        namespace='robot_1',
        output='screen',
        parameters=[{**common_params, 
                     'robot_base_frame': 'robot_1/base_footprint',
                     'map_topic': '/robot_1/global_costmap/costmap'}],
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
        parameters=[{**common_params, 
                     'robot_base_frame': 'robot_2/base_footprint',
                     'map_topic': '/robot_2/global_costmap/costmap'}],
        remappings=[
            ('navigate_to_pose', '/robot_2/navigate_to_pose'),
            *TF_REMAP,
        ],
    )

    delayed_frontiers = TimerAction(period=20.0, actions=[frontier_r1, frontier_r2])

    return LaunchDescription([
        *nav2_nodes_r1,
        nav2_nodes_r2,
        delayed_frontiers,
    ])