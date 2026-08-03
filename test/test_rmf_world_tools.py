#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/rmf_world_tools.py の自己テスト。rmf_demos が未インストールでも実行できる。

    cd ~/ros2_ws/src/multi_explore_mapping
    python3 test/test_rmf_world_tools.py
"""

import math
import os
import sys
import tempfile
import textwrap
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import rmf_world_tools as rwt  # noqa: E402


# rmf_demos_maps が生成する .world を模した合成データ
WORLD = textwrap.dedent('''\
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="airport_terminal">
    <include><uri>model://L1</uri></include>
    <include><name>tinyRobot1</name><uri>model://tinyRobot</uri><pose>1 2 0 0 0 0</pose></include>
    <include><name>caddy1</name><uri>model://Caddy</uri></include>
    <include><name>chair_1</name><uri>model://OfficeChairGrey</uri></include>
    <model name="coe_door">
      <link name="right"/>
      <joint name="right_joint" type="prismatic"/>
      <plugin name="door" filename="libdoor.so"><door name="coe_door"/></plugin>
    </model>
    <model name="lift1">
      <plugin name="lift" filename="liblift.so"/>
    </model>
    <model name="some_wall"><link name="w"/></model>
    <plugin name="toggle_charging" filename="libtoggle_charging.so"/>
    <plugin name="toggle_floors" filename="libtoggle_floors.so"/>
    <plugin name="keep_me" filename="libgazebo_ros_init.so"/>
  </world>
</sdf>
''')

MODEL_SDFS = {
    'tinyRobot': '<plugin filename="libslotcar.so"/>',
    'Caddy': '<plugin filename="libreadonly.so"/>',
    'OfficeChairGrey': '<visual name="v"/>',
    'L1': '<static>true</static>',
}


def make_fixture(root):
    src = os.path.join(root, 'airport_terminal.world')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(WORLD)

    mdir = os.path.join(root, 'models')
    for name, body in MODEL_SDFS.items():
        os.makedirs(os.path.join(mdir, name), exist_ok=True)
        with open(os.path.join(mdir, name, 'model.sdf'), 'w', encoding='utf-8') as f:
            f.write(f'<sdf>{body}</sdf>')

    ng = os.path.join(root, 'nav_graphs')
    os.makedirs(ng, exist_ok=True)
    nav = {
        'building_name': 'airport_terminal',
        'levels': {
            'L1': {'lanes': [],
                   'vertices': [[float(i), float(j), {'name': f'v_{i}_{j}'}]
                                for i in range(-10, 11, 2) for j in range(-6, 7, 2)]},
            'L2': {'lanes': [], 'vertices': [[0.0, 0.0, {}]]},
        },
    }
    with open(os.path.join(ng, '0.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(nav, f)

    return src, mdir, ng


def test_sanitize(src, mdir, root):
    dst = os.path.join(root, 'out.world')
    rep = rwt.sanitize_world(src, dst, search_dirs=[mdir])

    world = ET.parse(dst).getroot().find('world')
    models = [m.get('name') for m in world.findall('model')]
    includes = [(i.findtext('name'), i.findtext('uri')) for i in world.findall('include')]
    plugins = [p.get('filename') for p in world.findall('plugin')]

    assert rep['doors'] == ['coe_door'], rep
    assert rep['lifts'] == ['lift1'], rep
    assert sorted(rep['robots']) == ['caddy1 (Caddy)', 'tinyRobot1 (tinyRobot)'], rep
    assert sorted(rep['world_plugins']) == ['libtoggle_charging.so', 'libtoggle_floors.so'], rep

    # 建物・什器・非RMFプラグインは残る
    assert models == ['some_wall'], models
    assert plugins == ['libgazebo_ros_init.so'], plugins
    assert (None, 'model://L1') in includes, includes
    assert ('chair_1', 'model://OfficeChairGrey') in includes, includes
    print('OK  sanitize_world')

    # keep_* オプション
    rep2 = rwt.sanitize_world(src, dst, search_dirs=[mdir],
                              remove_doors=False, remove_lifts=False,
                              remove_robots=False, remove_world_plugins=False)
    assert rep2 == {'doors': [], 'lifts': [], 'robots': [], 'world_plugins': []}, rep2
    print('OK  sanitize_world (keep options)')


def test_inspect(src):
    info = rwt.inspect_world(src)
    assert info['doors'] == ['coe_door'], info
    assert info['lifts'] == ['lift1'], info
    assert info['models'] == 3, info
    assert info['includes'] == 4, info
    print('OK  inspect_world')


def test_nav_graph(ng):
    pts = rwt.load_nav_vertices(ng)
    # 頂点数が多い L1 が選ばれること（L2 の 1 点ではない）
    assert len(pts) == 11 * 7, len(pts)
    print('OK  load_nav_vertices')

    cluster = rwt.pick_spawn_points(pts, 4, 'cluster', 1.5)
    spread = rwt.pick_spawn_points(pts, 4, 'spread')
    assert len(cluster) == 4 and len(spread) == 4

    # cluster は min_sep を満たす
    for i, a in enumerate(cluster):
        for b in cluster[i + 1:]:
            assert math.hypot(a[0] - b[0], a[1] - b[1]) >= 1.5, (a, b)

    def span(ps):
        return max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in ps for b in ps)

    assert span(spread) > span(cluster), (span(spread), span(cluster))
    print(f'OK  pick_spawn_points  cluster_span={span(cluster):.1f}'
          f'  spread_span={span(spread):.1f}')


def test_edge_cases(root, ng):
    assert rwt.load_nav_vertices(os.path.join(root, 'does_not_exist')) == []
    assert rwt.pick_spawn_points([], 4) == []
    assert rwt.pick_spawn_points([(0.0, 0.0)], 0) == []
    pts = rwt.load_nav_vertices(ng)
    # 要求台数が多すぎても落ちない（取れるだけ返す）
    assert len(rwt.pick_spawn_points(pts, 999, 'cluster', 1.5)) <= len(pts)
    print('OK  edge cases')


def main():
    with tempfile.TemporaryDirectory() as root:
        src, mdir, ng = make_fixture(root)
        test_sanitize(src, mdir, root)
        test_inspect(src)
        test_nav_graph(ng)
        test_edge_cases(root, ng)
    print('\nALL TESTS PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
