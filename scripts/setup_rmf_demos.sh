#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# rmf_demos のマップだけを別ワークスペースにセットアップする。
#
#   bash scripts/setup_rmf_demos.sh
#
# オプション:
#   --ws <path>      ワークスペース (default: ~/rmf_ws)
#   --no-models      Fuel からの什器モデルDLを省略（速いが什器の無いワールドになる）
#   --worlds-only    clone 済み前提でビルドだけやり直す
#   -y               apt の確認プロンプトを省略
#
# 何をするか:
#   1. apt から rmf_building_map_tools を入れる（無ければソースビルドに切替）
#   2. ~/rmf_ws/src に rmf_demos を clone
#   3. rmf_demos_assets と rmf_demos_maps だけをビルド（RMF 本体は不要）
#   4. .world が生成されたか検証
# ---------------------------------------------------------------------------
set -euo pipefail

RMF_WS="${HOME}/rmf_ws"
NO_MODELS=0
WORLDS_ONLY=0
APT_YES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ws)          RMF_WS="$2"; shift 2 ;;
    --no-models)   NO_MODELS=1; shift ;;
    --worlds-only) WORLDS_ONLY=1; shift ;;
    -y)            APT_YES="-y"; shift ;;
    -h|--help)     sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "不明なオプション: $1" >&2; exit 1 ;;
  esac
done

info()  { printf '\n\033[1;34m==>\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
die()   { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 0. 前提チェック
# ---------------------------------------------------------------------------
[[ -n "${ROS_DISTRO:-}" ]] || die "ROS がセットアップされていません。'source /opt/ros/humble/setup.bash' を先に実行してください。"
command -v colcon >/dev/null || die "colcon が見つかりません: sudo apt install python3-colcon-common-extensions"
command -v git    >/dev/null || die "git が見つかりません: sudo apt install git"

BRANCH="${ROS_DISTRO}"
info "ROS_DISTRO=${ROS_DISTRO} / workspace=${RMF_WS} / branch=${BRANCH}"

if [[ "${AMENT_PREFIX_PATH:-}" == *"${RMF_WS}"* ]]; then
  warn "${RMF_WS} が既に source 済みのシェルです。新しい端末で実行するほうが安全です。"
fi

# ---------------------------------------------------------------------------
# 1. ビルドツール (rmf_building_map_tools)
# ---------------------------------------------------------------------------
BMT_PKG="ros-${ROS_DISTRO}-rmf-building-map-tools"
BUILD_BMT_FROM_SOURCE=0

if [[ ${WORLDS_ONLY} -eq 0 ]]; then
  info "共通の python 依存を導入"
  sudo apt-get update
  sudo apt-get install ${APT_YES} \
    python3-shapely python3-rtree python3-requests python3-yaml python3-pip

  info "${BMT_PKG} が apt にあるか確認"
  if apt-cache policy "${BMT_PKG}" 2>/dev/null | grep -q 'Candidate: [^(]'; then
    echo "  -> あり。apt で導入します。"
    sudo apt-get install ${APT_YES} "${BMT_PKG}"
  else
    echo "  -> 無し。rmf_traffic_editor をソースビルドします。"
    BUILD_BMT_FROM_SOURCE=1
  fi
fi

# ---------------------------------------------------------------------------
# 2. clone
# ---------------------------------------------------------------------------
mkdir -p "${RMF_WS}/src"
cd "${RMF_WS}/src"

clone_or_update() {
  local url="$1" dir="$2"
  if [[ -d "${dir}/.git" ]]; then
    echo "  ${dir}: 既に存在するのでスキップ"
  else
    info "clone ${dir} (${BRANCH})"
    git clone "${url}" -b "${BRANCH}" "${dir}" \
      || { warn "${BRANCH} ブランチが無いので main を使います"; git clone "${url}" -b main "${dir}"; }
  fi
}

if [[ ${WORLDS_ONLY} -eq 0 ]]; then
  clone_or_update https://github.com/open-rmf/rmf_demos.git rmf_demos
  if [[ ${BUILD_BMT_FROM_SOURCE} -eq 1 ]]; then
    clone_or_update https://github.com/open-rmf/rmf_traffic_editor.git rmf_traffic_editor
  fi
fi

[[ -d "${RMF_WS}/src/rmf_demos/rmf_demos_maps" ]] || die "rmf_demos が clone されていません"

# ---------------------------------------------------------------------------
# 3. ビルド
# ---------------------------------------------------------------------------
cd "${RMF_WS}"

if [[ ${BUILD_BMT_FROM_SOURCE} -eq 1 ]]; then
  info "rmf_building_map_tools の依存を rosdep で解決"
  # GUI (Qt) を含む rmf_traffic_editor 本体は建てないので、対象を絞る
  if command -v rosdep >/dev/null; then
    rosdep install --from-paths src/rmf_traffic_editor/rmf_building_map_tools \
                   --ignore-src -r -y || warn "rosdep が一部失敗しました。続行します。"
  else
    warn "rosdep が無いのでスキップします"
  fi

  info "rmf_building_map_tools をビルド"
  colcon build --packages-select rmf_building_map_tools
  # shellcheck disable=SC1091
  source "${RMF_WS}/install/setup.bash"
fi

command -v ros2 >/dev/null || die "ros2 コマンドが見つかりません"
if ! ros2 pkg prefix rmf_building_map_tools >/dev/null 2>&1; then
  die "rmf_building_map_tools が見つかりません。apt 導入後に新しい端末で source し直してから再実行してください。"
fi

CMAKE_ARGS=()
if [[ ${NO_MODELS} -eq 1 ]]; then
  warn "--no-models: Fuel からの什器DLを省略します（床・壁のみのワールドになります）"
  CMAKE_ARGS=(--cmake-args -DNO_DOWNLOAD_MODELS=ON)
fi

info "rmf_demos_assets / rmf_demos_maps をビルド"
echo "  初回は Gazebo Fuel からのモデルDLで 10〜20 分かかることがあります。"
echo "  ログが止まって見えてもDL中の場合が多いので、しばらく待ってください。"
colcon build --packages-select rmf_demos_assets rmf_demos_maps "${CMAKE_ARGS[@]}"

# ---------------------------------------------------------------------------
# 4. 検証
# ---------------------------------------------------------------------------
# shellcheck disable=SC1091
source "${RMF_WS}/install/setup.bash"

info "生成された .world を確認"
MAPS_SHARE="$(ros2 pkg prefix rmf_demos_maps)/share/rmf_demos_maps/maps"
FOUND=0
for w in airport_terminal office campus hotel clinic; do
  if [[ -f "${MAPS_SHARE}/${w}/${w}.world" ]]; then
    SIZE=$(du -h "${MAPS_SHARE}/${w}/${w}.world" | cut -f1)
    NMODELS=$(find "${MAPS_SHARE}/${w}/models" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | wc -l)
    printf '  \033[1;32mOK\033[0m  %-18s %6s  models=%s\n' "${w}" "${SIZE}" "${NMODELS}"
    FOUND=$((FOUND+1))
  else
    printf '  --  %-18s (未生成)\n' "${w}"
  fi
done

[[ ${FOUND} -gt 0 ]] || die "ワールドが1つも生成されていません。上のビルドログを確認してください。"

cat <<EOF

------------------------------------------------------------------
セットアップ完了。

次の3行を ~/.bashrc に追記してください (順番が重要):

  source /opt/ros/\${ROS_DISTRO}/setup.bash
  source ${RMF_WS}/install/setup.bash
  source \${HOME}/ros2_ws/install/setup.bash

新しい端末を開いて、以下で起動できます:

  ros2 run multi_explore_mapping rmf_world_tools.py inspect --world airport_terminal
  ros2 launch multi_explore_mapping simulation_world.launch.py world_type:=rmf
------------------------------------------------------------------
EOF
