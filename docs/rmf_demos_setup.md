# rmf_demos のワールドを multi_explore_mapping で使う

Open-RMF の `rmf_demos` に含まれる大規模ワールド（airport_terminal / office / campus 等）を、
本パッケージの `simulation_world.launch.py` から直接起動できるようにするための手順。

環境想定: **Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11**

---

## 1. なぜセットアップが必要か

`rmf_demos_maps` の `.world` は **リポジトリに入っていない**。
traffic-editor 形式の `*.building.yaml` から、**ビルド時に生成される**ためである。

```
maps/airport_terminal/airport_terminal.building.yaml
        ↓  colcon build (rmf_building_map_tools)
share/rmf_demos_maps/maps/airport_terminal/
        ├── airport_terminal.world     ← Gazebo に渡すのはこれ
        ├── models/                    ← 生成された床・壁 + Fuel からDLした什器
        └── nav_graphs/0.yaml          ← RMF の走行レーン（スポーン位置の算出に使う）
```

また Humble では `rmf_demos` のバイナリ配布が無いため、ソースビルドが前提になる。
ただし本パッケージが必要とするのは `rmf_demos_maps` と `rmf_demos_assets` の
**2 パッケージだけ**で、RMF 本体（フリートアダプタ、タスク配分など）は不要。

**このワークスペースは `~/ros2_ws` とは別に `~/rmf_ws` として作る。**
`rmf_demos_maps` のビルドは毎回 `.world` 再生成と Fuel からのDLを走らせるので、
`~/ros2_ws/src` に置くと以後 `colcon build` のたびに巻き込まれる。

---

## 2. インストール

### 2.0 スクリプトで一括（推奨）

```bash
source /opt/ros/humble/setup.bash
bash ~/ros2_ws/src/multi_explore_mapping/scripts/setup_rmf_demos.sh
```

`~/rmf_ws` の作成 → ビルドツールの導入 → clone → `rmf_demos_assets` /
`rmf_demos_maps` のビルド → 生成確認まで行う。ディレクトリは自動で作られるので
事前の `mkdir` は不要。途中で失敗しても再実行できる（clone 済みはスキップ）。

主なオプション:

| オプション | 意味 |
|---|---|
| `--ws <path>` | ワークスペースの場所（既定 `~/rmf_ws`） |
| `--no-models` | Fuel からの什器DLを省略（速いが床・壁のみ） |
| `--worlds-only` | clone 済み前提でビルドだけやり直す |
| `-y` | apt の確認プロンプトを省略 |

以下 2.1〜2.4 は、このスクリプトが内部でやっていることの手動版（参考）。

### 2.1 ビルドツール

```bash
sudo apt update
sudo apt install ros-humble-rmf-building-map-tools
sudo apt install python3-shapely python3-yaml python3-requests python3-rtree
```

`ros-humble-rmf-building-map-tools` が見つからない場合は
[rmf_traffic_editor](https://github.com/open-rmf/rmf_traffic_editor) の `humble` ブランチを
ソースで入れる:

```bash
mkdir -p ~/rmf_ws/src && cd ~/rmf_ws/src
git clone https://github.com/open-rmf/rmf_traffic_editor.git -b humble
cd ~/rmf_ws
colcon build --packages-select rmf_building_map_tools
source ~/rmf_ws/install/setup.bash
```

### 2.2 rmf_demos のマップだけビルド

```bash
cd ~/rmf_ws/src
git clone https://github.com/open-rmf/rmf_demos.git -b humble

cd ~/rmf_ws          # src の一つ上。src の中で colcon build すると失敗する
colcon build --packages-select rmf_demos_assets rmf_demos_maps
```

ビルド中に Gazebo Fuel から什器モデルをダウンロードするため、**ネットワーク接続が必要**。
`BUILDING WORLDFILE WITH COMMAND: ...` や `DOWNLOADING MODELS WITH COMMAND: ...` が
stderr に出るが、これは colcon の custom target のログであってエラーではない。

ダウンロードを省きたい場合は `--cmake-args -DNO_DOWNLOAD_MODELS=ON` を付けられるが、
什器が欠けたワールドになる。

### 2.3 source

`~/.bashrc` にこの順で書く:

```bash
source /opt/ros/humble/setup.bash
source ~/rmf_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

別ワークスペースでも問題なく参照できる。`source` するたびに `AMENT_PREFIX_PATH` が
追記され、`get_package_share_directory('rmf_demos_maps')` が実行時にそこを辿るだけだから。
本パッケージは rmf の share ディレクトリ内のファイルを読むだけで、ヘッダの include も
ライブラリのリンクもしていないので、**ビルド時の依存は無い**（rmf 無しでも
`colcon build` は通るし `world_type:=aws` は動く）。

### 2.4 本パッケージのリビルド

```bash
cd ~/ros2_ws
colcon build --packages-select multi_explore_mapping
source install/setup.bash
```

---

## 3. 動作確認

### 3.1 ツール単体のテスト（rmf_demos 不要）

合成データで変換ロジックだけを検証する。rmf_demos を入れる前に実行できる:

```bash
cd ~/ros2_ws/src/multi_explore_mapping
python3 test/test_rmf_world_tools.py
# -> ALL TESTS PASSED
```

### 3.2 実際のワールドの確認

```bash
ros2 run multi_explore_mapping rmf_world_tools.py inspect --world airport_terminal
ros2 run multi_explore_mapping rmf_world_tools.py paths   --world airport_terminal
```

---

## 4. 起動

```bash
# 既定 (airport_terminal, 2台, ドア除去あり)
ros2 launch multi_explore_mapping simulation_world.launch.py world_type:=rmf

# ワールドと台数を指定
ros2 launch multi_explore_mapping simulation_world.launch.py \
    world_type:=rmf rmf_world:=office num_robots:=4

# 分散スタート（探査効率の上限を見る用）
ros2 launch multi_explore_mapping simulation_world.launch.py \
    world_type:=rmf spawn_mode:=spread num_robots:=4

# 従来通り
ros2 launch multi_explore_mapping simulation_world.launch.py world_type:=aws
ros2 launch multi_explore_mapping simulation_world.launch.py world_type:=custom
```

### launch 引数

| 引数 | 既定値 | 説明 |
|---|---|---|
| `world_type` | `aws` | `aws` / `custom` / `rmf` |
| `rmf_world` | `airport_terminal` | `airport_terminal`, `office`, `campus`, `hotel`, `clinic` |
| `rmf_map_package` | `rmf_demos_maps` | 自作の traffic-editor マップパッケージに差し替え可 |
| `open_doors` | `true` | RMF のドア/リフト/ロボットを world から除去 |
| `num_robots` | `2` | スポーン台数 |
| `robot_model` | `burger` | `burger` / `waffle` / `waffle_pi` |
| `spawn_mode` | `cluster` | `cluster`: 近接スタート（地図マージ検証向き） / `spread`: 分散スタート |
| `spawn_min_sep` | `1.5` | cluster モードでのロボット間最小距離 [m] |

---

## 5. `open_doors` が何をしているか

`rmf_demos` のワールドのドアは

```xml
<model name="coe_door">
  <link name="right"/>
  <joint name="right_joint" type="prismatic"/>
  <plugin name="door" filename="libdoor.so"> ... </plugin>
</model>
```

という形で、`libdoor.so` が RMF の `/door_requests` トピックを待って動く。
RMF のノード群を起動しない限り**ドアは閉じたまま**で、探査が途中で詰まる。

`open_doors:=true`（既定）では、起動前に `scripts/rmf_world_tools.py` が
以下を **`<model>` ごと削除**した `.world` を `/tmp` に書き出し、それを Gazebo に渡す:

| 対象 | 判定方法 | 削除する理由 |
|---|---|---|
| ドア | `libdoor.so` プラグインを持つ `<model>` | 常時開放にする。ドア枠（壁）は別要素なので残る |
| リフト | `liblift.so` | 2D 探査では使わない |
| RMF ロボット | `<include>` 先の `model.sdf` が `slotcar` / `readonly` を含む | フリートアダプタ無しでは動かず障害物になるだけ |
| world プラグイン | `libtoggle_charging.so` 等 | RMF ノード不在時のエラーログを抑える |

副作用として、`rmf_building_sim_gz_classic_plugins` /
`rmf_robot_sim_gz_classic_plugins` を**インストールしなくても起動できる**。

元のままで起動したい場合は `open_doors:=false`。
その場合は上記プラグインパッケージが必要。

---

## 6. スポーン位置の決め方

`rmf_demos` のワールドは原点が建物の隅とは限らず、`(0, 0)` に置くと
壁の中や床の外に湧く可能性がある。

そこで、`rmf_demos_maps` がビルド時に生成する `nav_graphs/0.yaml`
（RMF の走行レーンのウェイポイント）を読み、**必ず床の上にある座標**から選んでいる。

- `spawn_mode:=cluster`（既定）… 建物中央付近のウェイポイントを種に、
  `spawn_min_sep` 以上離れた近傍を順に採用。AWS 倉庫での配置
  （`(0, 2)` と `(0, -2)`）に近い**近接スタート**。ICP による地図マージの検証向き。
- `spawn_mode:=spread` … farthest-point sampling で最大限ばらけさせる。
  初期位置が既知で分散配置したときの探査効率を見る用途。

座標だけ先に確認したい場合:

```bash
ros2 run multi_explore_mapping rmf_world_tools.py spawns --world airport_terminal -n 4 --mode cluster
```

`nav_graphs` が無い / PyYAML が無い場合は `DEFAULT_ROBOT_POSES`（`(0,±2)` など）に
フォールバックする。ログに警告が出るので、必要なら手動で座標を指定すること。

---

## 7. ワールド選定の指針（2D LiDAR 探査）

| ワールド | 層 | 規模 | 2D 探査への適性 |
|---|---|---|---|
| `airport_terminal` | 単層 | 最大級 | ◎ 開放空間＋通路が多く、フロンティア探査の評価に向く。ドアが少ない |
| `office` | 単層 | 中 | ○ 部屋＋廊下構成で倉庫に近い。ドアが多いので `open_doors:=true` 必須 |
| `campus` | 単層 | 最大 | △ 屋外・GPS 座標系ベース。2D SLAM には広すぎる場合がある |
| `hotel` / `clinic` | 複層 | 中 | × リフト前提。2D 単層探査には不向き |

---

## 8. トラブルシューティング

**`パッケージ 'rmf_demos_maps' が見つかりません`**
`~/rmf_ws/install/setup.bash` を source していないか、`colcon build` が失敗している。
`ros2 pkg prefix rmf_demos_maps` で確認。

**`already exists and is not an empty directory` (git clone)**
既に clone 済み。`git status` がクリーンならそのまま使ってよい。撮り直す必要はない。

**Gazebo が起動するが真っ白 / モデルが出ない**
`GAZEBO_MODEL_PATH` が通っていない。
`rmf_world_tools.py paths --world airport_terminal` で表示されるパスが実在するか確認。

**起動が非常に遅い / 途中で止まる**
airport_terminal は什器が多く、初回ロードに時間がかかる。
launch 側で `SPAWN_START` を rmf のとき 15 秒に伸ばしてあるが、
マシンによってはさらに延ばす必要がある（`launch/simulation_world.launch.py`）。

**ロボットが壁の中に湧く**
`spawns` サブコマンドで座標を確認する。想定と違う場合は
`nav_graphs` の座標系とワールド座標系がずれている可能性がある
（georeferenced なワールドで起きうる）。その場合は `DEFAULT_ROBOT_POSES` を
手動で書き換えるのが確実。

---

## 参考

- [rmf_demos](https://github.com/open-rmf/rmf_demos)
- [rmf (インストール手順)](https://github.com/open-rmf/rmf)
- [rmf_traffic_editor / rmf_building_map_tools](https://github.com/open-rmf/rmf_traffic_editor)
- [Programming Multiple Robots with ROS 2 — Demos](https://osrf.github.io/ros2multirobotbook/demos.html)
