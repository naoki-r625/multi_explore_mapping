#!/usr/bin/env python3
"""
PGM occupancy map → Gazebo Classic .world converter

Usage:
    python3 pgm_to_world.py --map edit_map.yaml --output ../worlds/edit_map_world.world
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

WALL_HEIGHT = 2.0     # [m]
MIN_AREA_PX = 2       # 小さい孤立ノイズを除去

# ────────────────────────────────────────────
# 1. マップ読み込み
# ────────────────────────────────────────────
def load_binary(yaml_path: str):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    resolution      = float(cfg['resolution'])
    origin          = cfg['origin']
    origin_x, origin_y = float(origin[0]), float(origin[1])
    occupied_thresh = float(cfg.get('occupied_thresh', 0.65))
    negate          = int(cfg.get('negate', 0))

    pgm_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), cfg['image'])
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"[ERROR] Cannot read image: {pgm_path}")

    H, W = img.shape

    if negate == 0:
        thresh_val = int((1.0 - occupied_thresh) * 255)
        binary = (img <= thresh_val).astype(np.uint8)
    else:
        thresh_val = int(occupied_thresh * 255)
        binary = (img >= thresh_val).astype(np.uint8)

    print(f"Map      : {W}×{H} px  ({W*resolution:.2f}×{H*resolution:.2f} m)")
    print(f"Origin   : ({origin_x}, {origin_y})")
    print(f"Occupied : {binary.sum()} px  ({100*binary.sum()/(W*H):.1f}%)")

    return binary, resolution, origin_x, origin_y


# ────────────────────────────────────────────
# 2. 行スキャン → 縦マージで最小矩形群に変換
# ────────────────────────────────────────────
def extract_wall_rects(binary: np.ndarray):
    """
    行ごとにランレングスを取得し、直前行と同一ランが続く間は縦に伸長する。
    Returns: list of (col_start, row_start, col_width, row_height) in pixels
    """
    H, W = binary.shape
    active = {}   # (c_start, c_width) → row_start
    rects  = []

    for r in range(H):
        row  = binary[r]
        runs = set()
        c = 0
        while c < W:
            if row[c]:
                start = c
                while c < W and row[c]:
                    c += 1
                run_w = c - start
                if run_w >= MIN_AREA_PX:
                    runs.add((start, run_w))
            else:
                c += 1

        # 前行から続かなかったランを確定
        ended = [key for key in active if key not in runs]
        for key in ended:
            rs = active.pop(key)
            rh = r - rs
            if rh >= MIN_AREA_PX:
                rects.append((key[0], rs, key[1], rh))

        # 新規ランを登録
        for key in runs:
            if key not in active:
                active[key] = r

    # 画像末尾まで続いたランを確定
    for (cs, cw), rs in active.items():
        rh = H - rs
        if rh >= MIN_AREA_PX:
            rects.append((cs, rs, cw, rh))

    return rects


# ────────────────────────────────────────────
# 3. ピクセル矩形 → ワールド座標
# ────────────────────────────────────────────
def rect_to_world(cs, rs, cw, rh, H, resolution, origin_x, origin_y):
    """ピクセル矩形 → (center_x, center_y, size_x, size_y)"""
    cx = origin_x + (cs + cw * 0.5) * resolution
    cy = origin_y + (H  - rs - rh * 0.5) * resolution
    sx = cw * resolution
    sy = rh * resolution
    return cx, cy, sx, sy


# ────────────────────────────────────────────
# 4. SDF 生成
# ────────────────────────────────────────────
def build_sdf(rects, H, resolution, origin_x, origin_y) -> str:
    cz = WALL_HEIGHT * 0.5
    sz = WALL_HEIGHT

    visuals    = []
    collisions = []
    for i, (cs, rs, cw, rh) in enumerate(rects):
        cx, cy, sx, sy = rect_to_world(cs, rs, cw, rh, H, resolution, origin_x, origin_y)
        pose = f"{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0"
        size = f"{sx:.4f} {sy:.4f} {sz:.4f}"
        visuals.append(f"""\
        <visual name="v{i}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>""")
        collisions.append(f"""\
        <collision name="c{i}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>""")

    bodies = "\n".join(visuals + collisions)

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="edit_map_world">

    <include>
      <uri>model://sun</uri>
    </include>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <model name="walls">
      <static>true</static>
      <link name="link">
{bodies}
      </link>
    </model>

  </world>
</sdf>
"""


# ────────────────────────────────────────────
# main
# ────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="PGM map → Gazebo .world")
    parser.add_argument("--map",    default="edit_map.yaml",
                        help="Path to the map YAML file")
    parser.add_argument("--output", default="../worlds/edit_map_world.world",
                        help="Output .world file path")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path  = os.path.join(script_dir, args.map)

    binary, resolution, origin_x, origin_y = load_binary(yaml_path)
    H, W = binary.shape

    rects = extract_wall_rects(binary)
    print(f"Wall rects: {len(rects)}")

    sdf = build_sdf(rects, H, resolution, origin_x, origin_y)

    output_path = os.path.join(script_dir, args.output)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(sdf)

    print(f"Written → {output_path}")

    # ロボット初期位置 (0,2), (0,-2) が自由空間か確認
    for wx, wy, name in [(0.0, 2.0, "robot_1"), (0.0, -2.0, "robot_2")]:
        px = int((wx - origin_x) / resolution)
        py = int(H - (wy - origin_y) / resolution)
        if 0 <= px < W and 0 <= py < H:
            val = binary[py, px]
            status = "OBSTACLE ⚠" if val else "free ✓"
            print(f"  Spawn check {name} ({wx},{wy}) → px=({px},{py}) {status}")
        else:
            print(f"  Spawn check {name} ({wx},{wy}) → OUT OF MAP ⚠")


if __name__ == "__main__":
    main()
