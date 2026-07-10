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
    if world_type == 'custom':
        world_path = os.path.join(pkg_my_mapping, 'worlds', 'my_custom_room.world')
        #
        robots = [
            ('robot_1', 'burger', 3.0, 0.0 ,0.0),
            #('robot_2', 'burger', -3.0, 0.0, 0.0),
        ]
    else:
        #
        world_path = os.path.join(
            get_package_share_directory('aws_robomaker_small_warehouse_world'),
            'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
        )
        robots = [
            ('robot_1', 'burger', 0.0, 2.0, 0.0),
            #('robot_2', 'burger', 0.0, -2.0, 0.0),
        ]

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

    robot_nodes = []

    for (ns, model, x, y, yaw) in robots:
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
        robot_nodes.append(
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

        #
        robot_nodes.append(
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
        robot_nodes.append(
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
        robot_nodes.append(
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
                    'minimum_note_score': 0.25,      # 0.55から引き下げ。ループ候補を厳しく弾きすぎないように　調整マッチングスコア閾値
                    'link_match_minimum_response_coarse': 0.1, #粗探索における相関値閾値
                    'link_scan_maximum_distance': 1.5, # 近くの壁とのマッチング精度向上 最大対応点探索距離

                    # 2. ループ検索範囲の最適化（AWS Warehouseのスケールに合わせる）
                    'loop_search_maximum_distance': 5.0,  # 4.0から少し拡大（オドメトリのズレをカバー）自己位置の不確かさの境界
                    'loop_match_minimum_chain_size': 5,    # 3から5へ。誤ったループ（誤マッチング）による地図の崩壊を防ぐ 時間的一貫性の検証閾値
                    'loop_search_space_dimension': 8.0,    # 探索サブマップのサイズ（8.0でOK） ローカルサブマップ（局所地図）の空間サイズ
                    'loop_match_maximum_variance_coarse': 0.4, #共分散・分散の許容閾値

                    # 3. グラフ登録（キーフレーム）の頻度調整
                    # ロボットが少し動いただけでグラフにノードを追加し、ループ検知のチャンスを増やす
                    'minimum_travel_distance': 0.1,        # 0.2から0.1へ短縮 キーフレーム生成の幾何学的閾値
                    'minimum_travel_heading': 0.1,         # 0.2から0.1へ短縮 キーフレーム生成の幾何学的閾値

                    # 4. 【重要】ループ閉鎖後の最適化（スキャンバッファとグラフ調整）
                    'scan_buffer_size': 10,                # 過去のスキャンを保持するバッファ数 スキャンウィンドウサイズ
                    'scan_buffer_max_num_lines': 50,
                    'correlation_search_space_dimension': 0.5, #コスト関数の平滑化カーネル
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.03,

                    # 5. 補正計算（Ceres Solver）のバックエンド設定 ポーズグラフ最適化（Pose Graph Optimization: PGO）の実行頻度
                    'loop_search_space_resolution': 0.05, #グラフ最適化のトリガー頻度
                    'optimize_every_n_nodes': 3,          # 3つノードが追加されるたびにグラフを最適化
                }],
                remappings=[
                    ('/map', f'/{ns}/map'),
                    ('/map_metadata', f'/{ns}/map_metadata')
                ]
            )
        )

    # ✅ 初期位置情報をTFで表現（map → robot_i/map を正確に繋ぐ）
    static_tf_nodes = []
    for (ns, model, x, y, yaw) in robots:
        static_tf_nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_map_to_{ns}_map',
                # 引数を8個のスタイル（x, y, z, yaw, pitch, roll, frame_id, child_frame_id）に変更
                # yaw（Z軸回転）をそのまま渡せるため、初期の向き（yaw）も完璧に反映されます
                arguments=[
                    '0.0', '0.0', '0.0',  # X, Y, Z
                    '0.0', '0.0', '0.0', # Yaw, Pitch, Roll
                    'map', f'{ns}/map'      # 親フレーム, 子フレーム
                ],
            )
        )

    # 10秒待ってから一斉起動
    delayed_robots = TimerAction(
        period=5.0,
        actions=robot_nodes
    )

    return [
        gzserver,
        gzclient,
        *static_tf_nodes,  # ✅ static_tf を先に起動
        delayed_robots,
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