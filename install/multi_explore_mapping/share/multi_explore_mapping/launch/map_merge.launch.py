import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    map_merge_node = Node(
        package='multirobot_map_merge',
        executable='map_merge',
        name='map_merge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'known_init_poses': True,       # init_poseを使う (False=自動推定)
            'merged_map_topic': '/map',
            'world_frame': 'map',
            # ロボットの初期位置 (Gazeboスポーン位置と一致させる)
            'robot_1/map_merge/init_pose_x':   3.0,
            'robot_1/map_merge/init_pose_y':   0.0,
            'robot_1/map_merge/init_pose_z':   0.0,
            'robot_1/map_merge/init_pose_yaw': 0.0,
            'robot_2/map_merge/init_pose_x':  -3.0,
            'robot_2/map_merge/init_pose_y':   0.0,
            'robot_2/map_merge/init_pose_z':   0.0,
            'robot_2/map_merge/init_pose_yaw': 0.0,
        }]
    )

    # マージmap(frame_id='map')とSLAMのローカルmap(robot_i/map)を繋ぐTF
    # init_poseと必ず同じ値にする
    static_tf_r1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_r1_map',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'robot_1/map'],
    )

    static_tf_r2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_r2_map',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'robot_2/map'],
    )

    return LaunchDescription([
        map_merge_node,
        static_tf_r1,
        static_tf_r2,
    ])
