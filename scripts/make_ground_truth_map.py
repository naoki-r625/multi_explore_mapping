#!/usr/bin/env python3
"""
Gazeboワールドから「TurtleBot3 burgerのLiDAR高さで切った」真値占有格子地図を作る。

ワールド(.world)が参照する各モデルの衝突メッシュ(COLLADA .DAE)を読み込み、
LiDARの走査面(既定 z=0.171m: burger の ray sensor の取り付け高さ)で水平に
切断した断面線分を格子に描画する。LiDARが実際に観測できる高さの障害物だけが
占有として残る(床・天井・低い/高い物は出ない)。

出力は nav2_map_server 形式 (pgm + yaml):
  0   = 占有
  254 = 自由 (シード点から障害物に遮られずに到達できる領域)
  205 = 未知 (壁の外側・物体内部など到達不能領域)

座標系はGazeboのワールド座標(=SDFのpose)。slam_toolboxのmap座標はロボットの
初期位置が原点になるので、比較時はロボットのスポーン位置ぶん平行移動が必要。

例:
  python3 make_ground_truth_map.py \
    --world ../../aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world \
    --models ../../aws-robomaker-small-warehouse-world/models \
    --out maps/no_roof_small_warehouse_gt
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose_matrix(text):
    v = [float(t) for t in (text or '0 0 0 0 0 0').split()]
    v += [0.0] * (6 - len(v))
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(v[3], v[4], v[5])
    m[:3, 3] = v[:3]
    return m


def strip_ns(root):
    for el in root.iter():
        if '}' in el.tag:
            el.tag = el.tag.split('}', 1)[1]
    return root


def load_dae_triangles(path):
    """COLLADAメッシュを (N,3,3) の三角形配列(単位: m, ノード変換適用済み)で返す。"""
    root = strip_ns(ET.parse(path).getroot())
    unit_el = root.find('./asset/unit')
    unit = float(unit_el.get('meter', '1.0')) if unit_el is not None else 1.0
    up_el = root.find('./asset/up_axis')
    up_axis = (up_el.text or 'Z_UP').strip() if up_el is not None else 'Z_UP'

    geoms = {}
    for g in root.findall('./library_geometries/geometry'):
        mesh = g.find('mesh')
        if mesh is None:
            continue
        sources = {}
        for s in mesh.findall('source'):
            fa = s.find('float_array')
            if fa is None:
                continue
            stride = int(s.find('./technique_common/accessor').get('stride', '3'))
            arr = np.array(fa.text.split(), dtype=float).reshape(-1, stride)
            sources[s.get('id')] = arr
        vert_src = {}
        for v in mesh.findall('vertices'):
            for i in v.findall('input'):
                if i.get('semantic') == 'POSITION':
                    vert_src[v.get('id')] = i.get('source')[1:]
        tris = []
        for prim in mesh.findall('triangles') + mesh.findall('polylist') + mesh.findall('polygons'):
            inputs = prim.findall('input')
            offsets = [int(i.get('offset', '0')) for i in inputs]
            stride = max(offsets) + 1
            vin = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            voff = int(vin.get('offset', '0'))
            pos = sources[vert_src[vin.get('source')[1:]]]
            if prim.tag == 'triangles':
                p = np.array(prim.find('p').text.split(), dtype=int).reshape(-1, stride)
                idx = p[:, voff].reshape(-1, 3)
            elif prim.tag == 'polylist':
                vcount = [int(c) for c in prim.find('vcount').text.split()]
                p = np.array(prim.find('p').text.split(), dtype=int).reshape(-1, stride)
                idx_list, k = [], 0
                for c in vcount:
                    face = p[k:k + c, voff]
                    for j in range(1, c - 1):
                        idx_list.append([face[0], face[j], face[j + 1]])
                    k += c
                idx = np.array(idx_list, dtype=int)
            else:
                idx_list = []
                for pe in prim.findall('p'):
                    a = np.array(pe.text.split(), dtype=int).reshape(-1, stride)[:, voff]
                    for j in range(1, len(a) - 1):
                        idx_list.append([a[0], a[j], a[j + 1]])
                idx = np.array(idx_list, dtype=int)
            if len(idx):
                tris.append(pos[idx])
        if tris:
            geoms['#' + g.get('id')] = np.concatenate(tris, axis=0)

    def node_matrix(node):
        m = np.eye(4)
        for ch in node:
            if ch.tag == 'matrix':
                m = m @ np.array(ch.text.split(), dtype=float).reshape(4, 4)
            elif ch.tag == 'translate':
                t = np.eye(4)
                t[:3, 3] = [float(x) for x in ch.text.split()]
                m = m @ t
            elif ch.tag == 'rotate':
                ax, ay, az, deg = [float(x) for x in ch.text.split()]
                a = np.array([ax, ay, az])
                a = a / (np.linalg.norm(a) or 1.0)
                th = math.radians(deg)
                k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                r = np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * (k @ k)
                t = np.eye(4)
                t[:3, :3] = r
                m = m @ t
            elif ch.tag == 'scale':
                s = np.eye(4)
                s[0, 0], s[1, 1], s[2, 2] = [float(x) for x in ch.text.split()]
                m = m @ s
        return m

    out = []

    def walk(node, parent):
        m = parent @ node_matrix(node)
        for ig in node.findall('instance_geometry'):
            tri = geoms.get(ig.get('url'))
            if tri is None:
                continue
            flat = np.hstack([tri.reshape(-1, 3), np.ones((tri.shape[0] * 3, 1))])
            out.append((flat @ m.T)[:, :3].reshape(-1, 3, 3))
        for child in node.findall('node'):
            walk(child, m)

    for vs in root.findall('./library_visual_scenes/visual_scene'):
        for n in vs.findall('node'):
            walk(n, np.eye(4))
    if not out:
        return np.zeros((0, 3, 3))
    tris = np.concatenate(out, axis=0) * unit
    if up_axis == 'Y_UP':
        tris = tris[:, :, [0, 2, 1]] * np.array([1, -1, 1])
    return tris


def resolve_uri(uri, model_dir, models_root):
    if uri.startswith('model://'):
        return os.path.join(models_root, uri[len('model://'):])
    if uri.startswith('file://'):
        rel = uri[len('file://'):]
        cand = os.path.join(os.path.dirname(models_root), rel)
        if os.path.exists(cand):
            return cand
        return os.path.join(models_root, rel.split('models/', 1)[-1])
    return os.path.join(model_dir, uri)


def model_collision_triangles(model_name, models_root, cache):
    """モデル座標系での全衝突メッシュ三角形 (N,3,3)。"""
    if model_name in cache:
        return cache[model_name]
    model_dir = os.path.join(models_root, model_name)
    sdf = strip_ns(ET.parse(os.path.join(model_dir, 'model.sdf')).getroot())
    model_el = sdf.find('model')
    m_model = pose_matrix((model_el.findtext('pose') or ''))
    result = []
    for link in model_el.findall('link'):
        m_link = m_model @ pose_matrix(link.findtext('pose') or '')
        for col in link.findall('collision'):
            m_col = m_link @ pose_matrix(col.findtext('pose') or '')
            mesh = col.find('./geometry/mesh')
            if mesh is None:
                print(f'  [skip] {model_name}: mesh以外のcollisionは未対応', file=sys.stderr)
                continue
            scale = [float(x) for x in (mesh.findtext('scale') or '1 1 1').split()]
            path = resolve_uri(mesh.findtext('uri'), model_dir, models_root)
            tri = load_dae_triangles(path) * np.array(scale)
            if len(tri) == 0:
                continue
            flat = np.hstack([tri.reshape(-1, 3), np.ones((tri.shape[0] * 3, 1))])
            result.append((flat @ m_col.T)[:, :3].reshape(-1, 3, 3))
    cache[model_name] = np.concatenate(result, axis=0) if result else np.zeros((0, 3, 3))
    return cache[model_name]


def slice_segments(tris, z):
    """三角形群を水平面 z で切った線分 (M,2,2) を返す。"""
    zs = tris[:, :, 2]
    mask = (zs.min(axis=1) <= z) & (zs.max(axis=1) >= z) & (zs.max(axis=1) - zs.min(axis=1) > 1e-9)
    segs = []
    for t in tris[mask]:
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            za, zb = t[a, 2] - z, t[b, 2] - z
            if za == 0 and zb == 0:
                continue
            if (za <= 0 <= zb) or (zb <= 0 <= za):
                if za == zb:
                    continue
                s = za / (za - zb)
                pts.append(t[a, :2] + s * (t[b, :2] - t[a, :2]))
        if len(pts) >= 2:
            segs.append([pts[0], pts[1]])
    return np.array(segs) if segs else np.zeros((0, 2, 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', required=True, help='model:// の解決先ディレクトリ')
    ap.add_argument('--out', required=True, help='出力ベース名 (.pgm/.yaml/.png を付けて出力)')
    ap.add_argument('--z', type=float, default=0.171, help='LiDAR走査面の高さ[m] (burger既定 0.171)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--seed', type=float, nargs=2, default=[0.0, 0.0],
                    help='自由空間とみなす起点のワールド座標 x y (ロボットのスポーン位置付近)')
    ap.add_argument('--margin', type=float, default=1.0)
    args = ap.parse_args()

    world = strip_ns(ET.parse(args.world).getroot())
    if world.tag == 'sdf' and world.find('world') is not None:
        world = world.find('world')
    cache = {}
    all_segs = []
    # <include> はワールド直下のものと、<model><include/><pose/></model> の形で
    # 包まれたもの(AWSワールドはこちら。poseは外側の<model>側にある)の両方がある
    placements = []
    for inc in world.findall('include'):
        placements.append((inc, pose_matrix(inc.findtext('pose') or '')))
    for mdl in world.findall('model'):
        outer = pose_matrix(mdl.findtext('pose') or '')
        for inc in mdl.findall('include'):
            placements.append((inc, outer @ pose_matrix(inc.findtext('pose') or '')))
    for inc, m in placements:
        uri = inc.findtext('uri')
        if not uri or not uri.startswith('model://'):
            continue
        name = uri[len('model://'):]
        tris = model_collision_triangles(name, args.models, cache)
        if len(tris) == 0:
            continue
        flat = np.hstack([tris.reshape(-1, 3), np.ones((tris.shape[0] * 3, 1))])
        wt = (flat @ m.T)[:, :3].reshape(-1, 3, 3)
        segs = slice_segments(wt, args.z)
        print(f'{name:55s} tris={len(tris):5d} segs@z={args.z}: {len(segs)}')
        if len(segs):
            all_segs.append(segs)
    if not all_segs:
        sys.exit('走査面と交差するコリジョンがありません (--z を確認)')
    segs = np.concatenate(all_segs, axis=0)

    res = args.resolution
    lo = segs.reshape(-1, 2).min(axis=0) - args.margin
    hi = segs.reshape(-1, 2).max(axis=0) + args.margin
    ox, oy = math.floor(lo[0] / res) * res, math.floor(lo[1] / res) * res
    w = int(math.ceil((hi[0] - ox) / res)) + 1
    h = int(math.ceil((hi[1] - oy) / res)) + 1

    occ = np.zeros((h, w), np.uint8)
    shift = 4
    k = 1 << shift
    for a, b in segs:
        p = [(int(round(((x - ox) / res) * k)), int(round((h - ((y - oy) / res)) * k))) for x, y in (a, b)]
        cv2.line(occ, p[0], p[1], 255, 1, cv2.LINE_8, shift)

    free_mask = (occ == 0).astype(np.uint8)
    sx = int((args.seed[0] - ox) / res)
    sy = h - 1 - int((args.seed[1] - oy) / res)
    if not (0 <= sx < w and 0 <= sy < h) or occ[sy, sx]:
        sys.exit(f'シード点 {args.seed} が範囲外か障害物上です。--seed を調整してください')
    ff = np.zeros((h + 2, w + 2), np.uint8)
    reach = free_mask.copy()
    cv2.floodFill(reach, ff, (sx, sy), 2, flags=4)
    img = np.full((h, w), 205, np.uint8)
    img[reach == 2] = 254
    img[occ == 255] = 0

    base = args.out
    os.makedirs(os.path.dirname(os.path.abspath(base)), exist_ok=True)
    cv2.imwrite(base + '.pgm', img)
    cv2.imwrite(base + '.png', img)
    with open(base + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(base)}.pgm\nresolution: {res}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'\n出力: {base}.pgm/.yaml/.png  size={w}x{h}  origin=({ox:.2f},{oy:.2f})  '
          f'free={int((img == 254).sum())} occ={int((img == 0).sum())}')


if __name__ == '__main__':
    main()
