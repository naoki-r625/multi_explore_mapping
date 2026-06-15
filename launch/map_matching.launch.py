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
            'robot_1_init_y':   2.0,
            'robot_1_init_yaw': 0.0,
            'robot_2_init_x':   0.0,
            'robot_2_init_y':   -2.0,
            'robot_2_init_yaw': 0.0,
            # ICP パラメータ
            'max_icp_iterations':        50,
            'icp_convergence_threshold': 0.05,  # メートル単位
            'max_feature_points':        2000,
        }]
    )

    # SLAM が安定してからマッチングを開始
    delayed_icp = TimerAction(
        period=10.0,
        actions=[icp_matching_node]
    )

    return LaunchDescription([delayed_icp])
