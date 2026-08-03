#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rmf_demos のワールドを「探査用の純粋な建物環境」として使うためのユーティリティ。

rmf_demos_maps が生成する .world には、以下の RMF 依存要素が含まれる:
  * ドア   : <model> + <plugin filename="libdoor.so">   → RMF ノード無しでは閉じたまま
  * リフト : <model> + <plugin filename="liblift.so">
  * ロボット: <include><uri>model://tinyRobot</uri>      → slotcar プラグイン依存
  * world プラグイン: libtoggle_charging.so / libtoggle_floors.so / libcrowd_simulator.so

このスクリプトは、それらを取り除いた .world を別ファイルとして書き出す。
床・壁・什器などの静的ジオメトリはそのまま残るので、2D LiDAR SLAM /
フロンティア探査の対象としてそのまま使える。

CLI 例:
    # 変換して出力（既定で doors/lifts/robots/world-plugins を除去）
    ros2 run multi_explore_mapping rmf_world_tools.py sanitize \
        --world airport_terminal -o /tmp/airport_open.world

    # 何が入っているか確認するだけ
    ros2 run multi_explore_mapping rmf_world_tools.py inspect --world airport_terminal

    # nav_graph からスポーン候補座標を出す
    ros2 run multi_explore_mapping rmf_world_tools.py spawns --world airport_terminal -n 4
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover
    get_package_share_directory = None


# ---------------------------------------------------------------------------
# 除去対象の定義
# ---------------------------------------------------------------------------

DOOR_PLUGINS = ('libdoor.so',)
LIFT_PLUGINS = ('liblift.so',)
# <include> 先の model.sdf に含まれていたら「RMF が動かすロボット」とみなす
ROBOT_PLUGIN_MARKERS = ('libslotcar.so', 'libreadonly.so', 'slotcar', 'readonly')
# <world> 直下に置かれる RMF 用プラグイン
RMF_WORLD_PLUGINS = (
    'libtoggle_charging.so',
    'libtoggle_floors.so',
    'libcrowd_simulator.so',
    'librmf_building_sim',
    'librmf_robot_sim',
)

DEFAULT_MAP_PACKAGE = 'rmf_demos_maps'
DEFAULT_WORLD = 'airport_terminal'


# ---------------------------------------------------------------------------
# パス解決
# ---------------------------------------------------------------------------

def _share(pkg: str):
    """パッケージの share ディレクトリ。未インストールなら None。"""
    if get_package_share_directory is None:
        return None
    try:
        return get_package_share_directory(pkg)
    except Exception:
        return None


def world_paths(world_name: str = DEFAULT_WORLD,
                map_package: str = DEFAULT_MAP_PACKAGE) -> dict:
    """rmf_demos_maps 内の各パスを返す。存在チェックはしない。"""
    share = _share(map_package)
    if share is None:
        raise RuntimeError(
            f"パッケージ '{map_package}' が見つかりません。"
            " rmf_demos をソースビルドして source しているか確認してください。")
    base = os.path.join(share, 'maps', world_name)
    return {
        'base': base,
        'world': os.path.join(base, f'{world_name}.world'),
        'models': os.path.join(base, 'models'),
        'nav_graphs': os.path.join(base, 'nav_graphs'),
        'config_resource': os.path.join(base, 'config_resource'),
    }


def model_dirs(world_name: str = DEFAULT_WORLD,
               map_package: str = DEFAULT_MAP_PACKAGE) -> list:
    """GAZEBO_MODEL_PATH に入れるべきディレクトリ列。"""
    dirs = []
    try:
        dirs.append(world_paths(world_name, map_package)['models'])
    except RuntimeError:
        pass
    assets = _share('rmf_demos_assets')
    if assets:
        dirs.append(os.path.join(assets, 'models'))
    return [d for d in dirs if os.path.isdir(d)]


def resource_dirs() -> list:
    """GAZEBO_RESOURCE_PATH に入れるべきディレクトリ列。"""
    dirs = []
    assets = _share('rmf_demos_assets')
    if assets:
        dirs.append(assets)
    return [d for d in dirs if os.path.isdir(d)]


# ---------------------------------------------------------------------------
# ワールド変換
# ---------------------------------------------------------------------------

def _plugin_filenames(elem) -> list:
    return [p.get('filename', '') for p in elem.iter('plugin')]


def _has_plugin(elem, needles) -> bool:
    return any(any(n in f for n in needles) for f in _plugin_filenames(elem))


def _include_model_name(inc):
    uri = (inc.findtext('uri') or '').strip()
    if uri.startswith('model://'):
        return uri[len('model://'):].strip('/')
    return None


def _model_sdf_matches(model_name: str, search_dirs, needles) -> bool:
    for d in search_dirs:
        path = os.path.join(d, model_name, 'model.sdf')
        if os.path.isfile(path):
            try:
                with open(path, encoding='utf-8', errors='ignore') as f:
                    text = f.read()
            except OSError:
                return False
            return any(n in text for n in needles)
    return False


def sanitize_world(src_path: str,
                   dst_path: str,
                   search_dirs=None,
                   remove_doors: bool = True,
                   remove_lifts: bool = True,
                   remove_robots: bool = True,
                   remove_world_plugins: bool = True) -> dict:
    """
    rmf_demos の .world を読み込み、RMF 依存要素を除いたものを dst_path に書く。

    ドアは「開ける」のではなく <model> ごと削除する。ドア枠（壁）は別要素なので
    残り、通路だけが開通する。2D LiDAR から見ると常時開放の出入口になる。

    Returns: 除去したものの一覧(dict)
    """
    search_dirs = list(search_dirs or [])

    tree = ET.parse(src_path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        raise RuntimeError(f'{src_path} に <world> 要素が見つかりません')

    report = {'doors': [], 'lifts': [], 'robots': [], 'world_plugins': []}

    for child in list(world):
        tag = child.tag

        if tag == 'model':
            name = child.get('name', '(no name)')
            if remove_doors and _has_plugin(child, DOOR_PLUGINS):
                world.remove(child)
                report['doors'].append(name)
                continue
            if remove_lifts and _has_plugin(child, LIFT_PLUGINS):
                world.remove(child)
                report['lifts'].append(name)
                continue
            if remove_robots and _has_plugin(child, ROBOT_PLUGIN_MARKERS):
                world.remove(child)
                report['robots'].append(name)
                continue

        elif tag == 'include':
            model_name = _include_model_name(child)
            if remove_robots and model_name and _model_sdf_matches(
                    model_name, search_dirs, ROBOT_PLUGIN_MARKERS):
                world.remove(child)
                label = child.findtext('name') or model_name
                report['robots'].append(f'{label} ({model_name})')
                continue

        elif tag == 'plugin':
            fname = child.get('filename', '')
            if remove_world_plugins and any(n in fname for n in RMF_WORLD_PLUGINS):
                world.remove(child)
                report['world_plugins'].append(fname)
                continue

    os.makedirs(os.path.dirname(os.path.abspath(dst_path)), exist_ok=True)
    tree.write(dst_path, encoding='utf-8', xml_declaration=True)
    return report


def inspect_world(src_path: str) -> dict:
    """除去せずに、どんな RMF 要素が入っているかだけ数える。"""
    tree = ET.parse(src_path)
    world = tree.getroot().find('world')
    out = {'doors': [], 'lifts': [], 'models': 0, 'includes': 0, 'world_plugins': []}
    if world is None:
        return out
    for child in world:
        if child.tag == 'model':
            out['models'] += 1
            if _has_plugin(child, DOOR_PLUGINS):
                out['doors'].append(child.get('name', '?'))
            if _has_plugin(child, LIFT_PLUGINS):
                out['lifts'].append(child.get('name', '?'))
        elif child.tag == 'include':
            out['includes'] += 1
        elif child.tag == 'plugin':
            out['world_plugins'].append(child.get('filename', '?'))
    return out


# ---------------------------------------------------------------------------
# スポーン地点（nav_graph 由来）
# ---------------------------------------------------------------------------

def load_nav_vertices(nav_graphs_dir: str, graph_idx: int = 0, level: str = None):
    """
    rmf_demos_maps が生成する nav_graphs/<idx>.yaml から
    ウェイポイント座標 [(x, y), ...] を読む。Gazebo のワールド座標系と一致する。
    """
    if yaml is None:
        return []
    path = os.path.join(nav_graphs_dir, f'{graph_idx}.yaml')
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return []

    levels = data.get('levels', data)
    if not isinstance(levels, dict) or not levels:
        return []

    if level and level in levels:
        node = levels[level]
    else:
        # 最もウェイポイントが多いレベル（＝1階相当）を選ぶ
        node = max(
            (v for v in levels.values() if isinstance(v, dict)),
            key=lambda v: len(v.get('vertices', []) or []),
            default=None)
    if not isinstance(node, dict):
        return []

    pts = []
    for v in node.get('vertices', []) or []:
        try:
            pts.append((float(v[0]), float(v[1])))
        except (TypeError, ValueError, IndexError):
            continue
    return pts


def pick_spawn_points(points, n: int, mode: str = 'cluster', min_sep: float = 1.5):
    """
    ウェイポイント集合から n 個のスポーン地点を選ぶ。

    mode='cluster' : 中央付近の 1 点を種にして、min_sep 以上離れた近傍を順に採用。
                     → 全機が近接スタート。地図マージ / ICP の検証向き。
    mode='spread'  : farthest-point sampling で最大限ばらけさせる。
                     → 分散配置での探査効率の上限を見る用途向き。
    """
    pts = list(points)
    if not pts or n <= 0:
        return []

    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    seed = min(pts, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
    chosen = [seed]

    if mode == 'spread':
        while len(chosen) < n:
            best, best_d = None, -1.0
            for p in pts:
                d = min(math.hypot(p[0] - c[0], p[1] - c[1]) for c in chosen)
                if d > best_d:
                    best, best_d = p, d
            if best is None or best_d < 1e-6:
                break
            chosen.append(best)
    else:
        ordered = sorted(pts, key=lambda p: math.hypot(p[0] - seed[0], p[1] - seed[1]))
        for p in ordered:
            if len(chosen) >= n:
                break
            if all(math.hypot(p[0] - c[0], p[1] - c[1]) >= min_sep for c in chosen):
                chosen.append(p)

    return chosen[:n]


def spawn_poses(world_name: str = DEFAULT_WORLD,
                map_package: str = DEFAULT_MAP_PACKAGE,
                n: int = 2,
                mode: str = 'cluster',
                min_sep: float = 1.5):
    """(x, y, yaw) のリストを返す。nav_graph が無ければ空リスト。"""
    try:
        paths = world_paths(world_name, map_package)
    except RuntimeError:
        return []
    pts = load_nav_vertices(paths['nav_graphs'])
    return [(x, y, 0.0) for (x, y) in pick_spawn_points(pts, n, mode, min_sep)]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _fmt_report(report: dict) -> str:
    lines = []
    for key, label in (('doors', 'ドア'), ('lifts', 'リフト'),
                       ('robots', 'RMFロボット'), ('world_plugins', 'worldプラグイン')):
        items = report.get(key, [])
        lines.append(f'  {label:<16} {len(items):>3} 件'
                     + (f'  -> {", ".join(map(str, items[:8]))}'
                        + (' ...' if len(items) > 8 else '') if items else ''))
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    def common(p):
        p.add_argument('--world', default=DEFAULT_WORLD,
                       help='rmf_demos のワールド名 (default: %(default)s)')
        p.add_argument('--map-package', default=DEFAULT_MAP_PACKAGE)

    p_ins = sub.add_parser('inspect', help='ワールドの中身を確認する')
    common(p_ins)

    p_san = sub.add_parser('sanitize', help='RMF 依存要素を除いた .world を書き出す')
    common(p_san)
    p_san.add_argument('-o', '--output', required=True)
    p_san.add_argument('--keep-doors', action='store_true')
    p_san.add_argument('--keep-lifts', action='store_true')
    p_san.add_argument('--keep-robots', action='store_true')

    p_spn = sub.add_parser('spawns', help='nav_graph からスポーン候補を出す')
    common(p_spn)
    p_spn.add_argument('-n', '--num', type=int, default=2)
    p_spn.add_argument('--mode', choices=('cluster', 'spread'), default='cluster')
    p_spn.add_argument('--min-sep', type=float, default=1.5)

    p_env = sub.add_parser('paths', help='GAZEBO_* に設定すべきパスを表示')
    common(p_env)

    args = parser.parse_args(argv)

    try:
        paths = world_paths(args.world, args.map_package)
    except RuntimeError as e:
        print(f'ERROR: {e}', file=sys.stderr)
        return 1

    if not os.path.isfile(paths['world']):
        print(f"ERROR: ワールドが見つかりません: {paths['world']}\n"
              f"       rmf_demos_maps のビルド時に生成されます。"
              f" colcon build が成功しているか確認してください。", file=sys.stderr)
        return 1

    if args.cmd == 'inspect':
        info = inspect_world(paths['world'])
        print(f"world: {paths['world']}")
        print(f"  <model>   {info['models']}")
        print(f"  <include> {info['includes']}")
        print(f"  ドア      {len(info['doors'])}  {info['doors'][:10]}")
        print(f"  リフト    {len(info['lifts'])}  {info['lifts'][:10]}")
        print(f"  world plugins {info['world_plugins']}")
        return 0

    if args.cmd == 'sanitize':
        report = sanitize_world(
            paths['world'], args.output,
            search_dirs=model_dirs(args.world, args.map_package),
            remove_doors=not args.keep_doors,
            remove_lifts=not args.keep_lifts,
            remove_robots=not args.keep_robots)
        print(f"in : {paths['world']}")
        print(f"out: {args.output}")
        print('除去:')
        print(_fmt_report(report))
        return 0

    if args.cmd == 'spawns':
        poses = spawn_poses(args.world, args.map_package,
                            args.num, args.mode, args.min_sep)
        if not poses:
            print('nav_graph からスポーン候補を取得できませんでした '
                  '(PyYAML 未導入 / nav_graphs 未生成)', file=sys.stderr)
            return 1
        for i, (x, y, yaw) in enumerate(poses, start=1):
            print(f'robot_{i}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
        return 0

    if args.cmd == 'paths':
        print('GAZEBO_MODEL_PATH   :', ':'.join(model_dirs(args.world, args.map_package)))
        print('GAZEBO_RESOURCE_PATH:', ':'.join(resource_dirs()))
        print('world               :', paths['world'])
        print('nav_graphs          :', paths['nav_graphs'])
        return 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
