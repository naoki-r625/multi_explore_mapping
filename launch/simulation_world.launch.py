import glob
import os
import sys
import tempfile

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# aws / custom ワールド用の既定スポーン配置 (先頭 num_robots 台を使う)
DEFAULT_ROBOT_POSES = [
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    #(0.0,  2.0, 0.0),
    #(0.0, -2.0, 0.0),
    #(-10.0,-10.0,0.0),
    #(-10.0,-15.0,0.0),
    (2.0,  0.0, 0.0),
    (-2.0, 0.0, 0.0),
    (2.0,  2.0, 0.0),
    (2.0, -2.0, 0.0),
]


def _prepend_env(var, paths):
    """環境変数の先頭にパスを足す (重複は除く)。"""
    paths = [p for p in paths if p]
    if not paths:
        return
    existing = [p for p in os.environ.get(var, '').split(':') if p]
    merged = paths + [p for p in existing if p not in paths]
    os.environ[var] = ':'.join(merged)


def _load_rmf_tools(pkg_my_mapping):
    """share/<pkg>/scripts/rmf_world_tools.py を import する。"""
    scripts_dir = os.path.join(pkg_my_mapping, 'scripts')
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    import rmf_world_tools  # noqa: E402
    return rmf_world_tools


def _setup_rmf_world(context, pkg_my_mapping, logs):
    """
    rmf_demos のワールドを使えるように前処理する。

    Returns: (world_path, robot_poses or None)
    """
    rmf_world = LaunchConfiguration('rmf_world').perform(context)
    map_package = LaunchConfiguration('rmf_map_package').perform(context)
    open_doors = LaunchConfiguration('open_doors').perform(context).lower() in ('1', 'true', 'yes')
    spawn_mode = LaunchConfiguration('spawn_mode').perform(context)
    spawn_min_sep = float(LaunchConfiguration('spawn_min_sep').perform(context))
    num_robots = int(LaunchConfiguration('num_robots').perform(context))

    tools = _load_rmf_tools(pkg_my_mapping)
    paths = tools.world_paths(rmf_world, map_package)

    if not os.path.isfile(paths['world']):
        raise RuntimeError(
            f"rmf_demos のワールドが見つかりません: {paths['world']}\n"
            "  rmf_demos_maps はビルド時に .world を生成します。\n"
            "  rmf_demos をソースビルドして setup.bash を source してください。")

    # --- Gazebo の探索パス -------------------------------------------------
    gazebo_share = None
    for cand in sorted(glob.glob('/usr/share/gazebo-*'), reverse=True):
        if os.path.isdir(cand):
            gazebo_share = cand
            break

    model_dirs = tools.model_dirs(rmf_world, map_package)
    if gazebo_share:
        model_dirs.append(os.path.join(gazebo_share, 'models'))
    _prepend_env('GAZEBO_MODEL_PATH', model_dirs)

    resource_dirs = tools.resource_dirs()
    if gazebo_share:
        resource_dirs.append(gazebo_share)
    _prepend_env('GAZEBO_RESOURCE_PATH', resource_dirs)

    # RMF プラグインがあれば読めるようにしておく (open_doors:=false のとき用)
    plugin_dirs = []
    for pkg, sub in (('rmf_robot_sim_gz_classic_plugins', 'lib/rmf_robot_sim_gz_classic_plugins'),
                     ('rmf_building_sim_gz_classic_plugins', 'lib/rmf_building_sim_gz_classic_plugins')):
        try:
            from ament_index_python.packages import get_package_prefix
            plugin_dirs.append(os.path.join(get_package_prefix(pkg), sub))
        except Exception:
            pass
    _prepend_env('GAZEBO_PLUGIN_PATH', plugin_dirs)

    # オンラインモデルDBへの問い合わせで固まるのを防ぐ
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    # --- ワールドの前処理 ---------------------------------------------------
    world_path = paths['world']
    if open_doors:
        out_dir = os.path.join(tempfile.gettempdir(), 'multi_explore_mapping_worlds')
        world_path = os.path.join(out_dir, f'{rmf_world}_open.world')
        report = tools.sanitize_world(
            paths['world'], world_path,
            search_dirs=tools.model_dirs(rmf_world, map_package))
        logs.append(LogInfo(msg=(
            f"[rmf] {rmf_world}: ドア {len(report['doors'])} / リフト {len(report['lifts'])} / "
            f"RMFロボット {len(report['robots'])} / worldプラグイン {len(report['world_plugins'])} を除去 "
            f"-> {world_path}")))
    else:
        logs.append(LogInfo(msg=f'[rmf] {rmf_world}: 元のワールドをそのまま使用 (ドアは閉じたまま)'))

    # --- スポーン位置 -------------------------------------------------------
    poses = tools.spawn_poses(rmf_world, map_package, num_robots, spawn_mode, spawn_min_sep)
    if poses and len(poses) < num_robots:
        logs.append(LogInfo(msg=(
            f'[rmf] nav_graph から {len(poses)} 点しか取得できませんでした '
            f'(要求 {num_robots} 台)。spawn_min_sep を小さくするか spawn_mode:=spread を試してください。'
            ' 既定座標にフォールバックします。')))
        poses = None
    if poses:
        logs.append(LogInfo(msg=(
            f"[rmf] nav_graph から {len(poses)} 台分のスポーン位置を取得 (mode={spawn_mode}): "
            + ', '.join(f'({x:.1f}, {y:.1f})' for x, y, _ in poses))))
    else:
        logs.append(LogInfo(msg=(
            '[rmf] nav_graph からスポーン位置を取得できませんでした。'
            ' 既定座標を使います (PyYAML 未導入 or nav_graphs 未生成)')))
        poses = None

    # --- GUIカメラをスポーン地点へ ------------------------------------------
    # open_doors:=false のときは world_path が元の(共有された)ワールドファイル
    # そのものなので書き換えない。open_doors:=true (既定)のときだけ、サニタイズ
    # 済みの自分専用コピーに対してカメラ位置を上書きする。
    if open_doors and poses:
        cx = sum(p[0] for p in poses) / len(poses)
        cy = sum(p[1] for p in poses) / len(poses)
        tools.point_camera_at(world_path, cx, cy)
        logs.append(LogInfo(msg=(
            f'[rmf] GUIカメラをスポーン地点付近 ({cx:.1f}, {cy:.1f}) の真上に移動しました')))

    return world_path, poses


def launch_setup(context, *args, **kwargs):
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_mapping = get_package_share_directory('multi_explore_mapping')

    world_type = LaunchConfiguration('world_type').perform(context)
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    robot_model = LaunchConfiguration('robot_model').perform(context)

    logs = []

    # 自作モデル (my_custom_model) を探索対象に入れる
    _prepend_env('GAZEBO_MODEL_PATH', [pkg_my_mapping])

    # ------------------------------------------------------------------
    # ワールドの選択
    # ------------------------------------------------------------------
    poses = None
    if world_type == 'custom':
        world_path = os.path.join(pkg_my_mapping, 'worlds', 'my_custom_room.world')
    elif world_type == 'edit_map':
        world_path = os.path.join(pkg_my_mapping, 'worlds', 'edit_map_world.world')
    elif world_type == 'rmf':
        world_path, poses = _setup_rmf_world(context, pkg_my_mapping, logs)
    else:  # 'aws'
        world_path = os.path.join(
            get_package_share_directory('aws_robomaker_small_warehouse_world'),
            'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world'
        )

    if poses is None:
        poses = DEFAULT_ROBOT_POSES[:num_robots]
        if len(poses) < num_robots:
            raise RuntimeError(
                f'num_robots={num_robots} 台分の既定スポーン座標がありません '
                f'(最大 {len(DEFAULT_ROBOT_POSES)})')

    robots = [(f'robot_{i + 1}', robot_model, x, y, yaw)
              for i, (x, y, yaw) in enumerate(poses)]

    # 1. Gazebo Server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Gazebo Client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_timers = []  # one TimerAction per robot, staggered by SPAWN_INTERVAL

    SPAWN_START    = 2.0   # [s] wait for Gazebo to finish loading the world
    SPAWN_INTERVAL = 1.0   # [s] gap between each robot's spawn + SLAM start

    # rmf_demos のワールドはロード自体が重いので待ち時間を伸ばす
    if world_type == 'rmf':
        SPAWN_START = 15.0

    all_namespaces = [r[0] for r in robots]

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
                }],
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

        # 他ロボットをスキャンから除去するゴーストフィルター
        peer_ns = [n for n in all_namespaces if n != ns]
        per_robot.append(
            Node(
                package='multi_explore_mapping',
                executable='ghost_filter_node',
                name='ghost_filter',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'peer_namespaces': peer_ns,
                    'mask_radius': 0.3,
                }],
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
                    'scan_topic': f'/{ns}/scan_filtered',
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
                    ('/map_metadata', f'/{ns}/map_metadata'),
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
        *logs,
        gzserver,
        gzclient,
        *static_tf_nodes,   # static_tf を先に起動
        *robot_timers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_type',
            default_value='aws',
            description='Select world environment: [aws, custom, edit_map, rmf]'
        ),
        DeclareLaunchArgument(
            'rmf_world',
            default_value='airport_terminal',
            description='world_type:=rmf のときの rmf_demos ワールド名 '
                        '[airport_terminal, office, campus, hotel, clinic]'
        ),
        DeclareLaunchArgument(
            'rmf_map_package',
            default_value='rmf_demos_maps',
            description='rmf_demos のマップパッケージ名'
        ),
        DeclareLaunchArgument(
            'open_doors',
            default_value='true',
            description='RMF のドア/リフト/ロボットを world から除去して常時開放にする'
        ),
        DeclareLaunchArgument(
            'num_robots',
            default_value='2',
            description='スポーンするロボット台数'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='burger',
            description='TurtleBot3 モデル [burger, waffle, waffle_pi]'
        ),
        DeclareLaunchArgument(
            'spawn_mode',
            default_value='cluster',
            description='rmf ワールドでのスポーン配置 '
                        '[cluster: 近接スタート(地図マージ向き) | spread: 分散スタート]'
        ),
        DeclareLaunchArgument(
            'spawn_min_sep',
            default_value='1.5',
            description='cluster モードでのロボット間最小距離 [m]'
        ),
        OpaqueFunction(function=launch_setup)
    ])
