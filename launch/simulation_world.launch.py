import os
import tempfile
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    #
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_mapping = get_package_share_directory('multi_explore_mapping')

    #
    world_type = LaunchConfiguration('world_type').perform(context)

    #
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{pkg_my_mapping}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_my_mapping

    #
    common_robots = [
        ('robot_1', 'burger', 0.0,  2.0, 0.0),
        ('robot_2', 'burger', 0.0, -2.0, 0.0),
        #('robot_3', 'burger', 2.0,  0.0, 0.0),
        #('robot_4', 'burger',-2.0,  0.0, 0.0),
        #('robot_5', 'burger', 2.0,  2.0, 0.0),
        #('robot_6', 'burger', 2.0, -2.0, 0.0),
    ]

    if world_type == 'custom':
        world_path = os.path.join(pkg_my_mapping, 'worlds', 'my_custom_room.world')
        robots = common_robots
    else:
        world_path = os.path.join(
            get_package_share_directory('aws_robomaker_small_warehouse_world'),
            'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
        )
        robots = common_robots

    # 1. Gazebo Server の起動 (AWS Warehouseワールド)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Gazebo Client (GUI) の起動
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_timers = []  # one TimerAction per robot, staggered by SPAWN_INTERVAL

    SPAWN_START    = 2.0   # [s] wait for Gazebo to finish loading the world
    SPAWN_INTERVAL = 1.0   # [s] gap between each robot's spawn + SLAM start

    for i, (ns, model, x, y, yaw) in enumerate(robots):
        # SDFモデルの動的書き換え
        sdf_path = os.path.join(pkg_tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
        with open(sdf_path, 'r') as f:
            sdf_content = f.read()

        sdf_content = sdf_content.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{ns}/odom</odometry_frame>')
        sdf_content = sdf_content.replace('<robot_base_frame>base_footprint</robot_base_frame>', f'<robot_base_frame>{ns}/base_footprint</robot_base_frame>')
        sdf_content = sdf_content.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{ns}/base_scan</frame_name>')
        sdf_content = sdf_content.replace('<frame_name>base_footprint</frame_name>', f'<frame_name>{ns}/base_footprint</frame_name>')
        sdf_content = sdf_content.replace('<visualize>true</visualize>', '<visualize>false</visualize>')

        tmp_sdf = tempfile.NamedTemporaryFile(mode='w', suffix=f'_{ns}.sdf', delete=False)
        tmp_sdf.write(sdf_content)
        tmp_sdf.close()

        # Robot State Publisher
        urdf_path = os.path.join(pkg_tb3_gazebo, 'urdf', f'turtlebot3_{model}.urdf')
        per_robot = []
        per_robot.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'frame_prefix': f'{ns}/',
                    'robot_description': open(urdf_path).read(),
                }]
            )
        )

        per_robot.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )

        # Gazeboへのスポーン
        per_robot.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{ns}',
                arguments=[
                    '-entity', ns,
                    '-file', tmp_sdf.name,
                    '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw),
                    '-robot_namespace', ns,
                ],
                output='screen'
            )
        )

        # 各ロボット専用の SLAM (slam_toolbox) ノード
        per_robot.append(
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'base_frame': f'{ns}/base_footprint',
                    'odom_frame': f'{ns}/odom',
                    'map_frame': f'{ns}/map',
                    'scan_topic': f'/{ns}/scan',
                    'mode': 'mapping',
                    'transform_timeout': 0.2,
                    'minimum_time_interval': 0.1,

                    # ===== ループクロージャ基本制御 =====
                    'do_loop_closing': True,
                    'map_update_interval': 1.0,

                    # 1. スキャンマッチングの厳格化（誤認識を防ぐ）
                    # 'minimum_note_score': 0.55,                    # ← 削除：存在しないパラメータ（無効）
                    # 'link_match_minimum_response_coarse': 0.1,     # ← 削除：存在しないパラメータ（無効）
                    'link_match_minimum_response_fine': 0.2,         # ← 追加：正しい名前。デフォルト0.1よりやや厳格化
                    'link_scan_maximum_distance': 1.5,

                    # 2. ループ検索範囲の最適化
                    'loop_search_maximum_distance': 1.5,             # 4.0 → 縮小（隣の棚に迷い込まないように）
                    'loop_match_minimum_chain_size': 10,             # 5 → 10に戻す（短いチェーンでの誤検出防止）
                    'loop_search_space_dimension': 2.0,
                    'loop_match_maximum_variance_coarse': 0.55,       # そのままでOK（厳格化に効いている）
                    'loop_match_minimum_response_coarse': 0.45,      # ← 追加：抜けていた本命パラメータ
                    'loop_match_minimum_response_fine': 0.65,        # ← 追加：抜けていた本命パラメータ

                    # 3. グラフ登録（キーフレーム）の頻度調整
                    'minimum_travel_distance': 0.1,
                    'minimum_travel_heading': 0.1,

                    # 4. ループ閉鎖後の最適化
                    'scan_buffer_size': 10,
                    # 'scan_buffer_max_num_lines': 50,               # ← 削除：存在しないパラメータ（無効）
                    'correlation_search_space_dimension': 0.5,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.03,

                    # 5. Ceres Solver バックエンド設定
                    'loop_search_space_resolution': 0.05,
                    'optimize_every_n_nodes': 3,
                }],
                remappings=[
                    ('/map', f'/{ns}/map'),
                    ('/map_metadata', f'/{ns}/map_metadata')
                ]
            )
        )

        # ロボットごとに時間差でスポーン（レースコンディション防止）
        robot_timers.append(
            TimerAction(
                period=SPAWN_START + i * SPAWN_INTERVAL,
                actions=per_robot
            )
        )

    #  初期位置情報をTFで表現（map → robot_i/map を正確に繋ぐ）
    static_tf_nodes = []
    for (ns, model, x, y, yaw) in robots:
        static_tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_map_to_{ns}_map',
                arguments=[
                    '0.0', '0.0', '0.0',  # X, Y, Z
                    '0.0', '0.0', '0.0',  # Yaw, Pitch, Roll
                    'map', f'{ns}/map'     # 親フレーム, 子フレーム
                ],
            )
        )

    return [
        gzserver,
        gzclient,
        *static_tf_nodes,   # static_tf を先に起動
        *robot_timers,      # robot_1: 5s, robot_2: 9s, ..., robot_6: 25s
    ]

def generate_launch_description():
    return LaunchDescription([
        # 引数の宣言 (aws か custom を指定可能。デフォルトは aws)
        DeclareLaunchArgument(
            'world_type',
            default_value='aws',
            description='Select world environment: [aws, custom]'
        ),
        # 上記の関数をコンテキスト付きで呼び出す
        OpaqueFunction(function=launch_setup)
    ])