from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    icp_matching_node = Node(
        package='multi_explore_mapping',
        executable='icp_map_matching_node',
        name='icp_matching',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # ロボット初期位置 (simulation_world.launch.py の spawn 位置と一致させる)
            # robot_1: (3, 0), robot_2: (-3, 0)
            'robot_1_init_x':   0.0,
            'robot_1_init_y':   0.0,
            'robot_1_init_yaw': 0.0,
            'robot_2_init_x':   0.0,
            'robot_2_init_y':   0.0,
            'robot_2_init_yaw': 0.0,
            # ICP パラメータ
            'max_icp_iterations':        50,
            'icp_convergence_threshold': 0.05,  # メートル単位
            'max_feature_points':        2000,
            # 特徴点の空間均一化 (ボクセルグリッドフィルタ)
            # 壁密集部への偏りを防ぎ ICP 精度を改善。0.1〜0.25m が目安。
            'voxel_size':                0.15,
            # 性能パラメータ: マージ書き込み用縮小倍率と統合周期
            # ※特徴点抽出は元解像度で行うためこの値は ICP 精度に影響しない
            'map_downsample_factor':     1,    # 解像度を2倍粗くして書き込みコストを1/4に
            'target_period_sec':         2.0,  # 処理が長引いても詰まらないよう自己再スケジュール
        }]
    )

    # SLAM が安定してからマッチングを開始
    delayed_icp = TimerAction(
        period=10.0,
        actions=[icp_matching_node]
    )

    return LaunchDescription([delayed_icp])
