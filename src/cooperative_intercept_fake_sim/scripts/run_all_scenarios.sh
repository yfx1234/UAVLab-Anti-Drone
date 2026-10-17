#!/usr/bin/env bash
# 快速运行全部 4 个场景（尽可能快的墙钟速度），验证四种典型协同场景。
# 用法：bash scripts/run_all_scenarios.sh   （在 package 根目录或任意位置执行）
set -e
cd "$(dirname "$0")/.."          # 切到 package 根目录
PKG_SRC=$(pwd)

if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1090
  source /opt/ros/humble/setup.bash
fi
if [ ! -f "$PKG_SRC/../../install/setup.bash" ]; then
  echo "未找到 ~/UAV_ws/install，请先执行: cd ~/UAV_ws && colcon build"
  exit 1
fi
# shellcheck disable=SC1090
source "$PKG_SRC/../../install/setup.bash"

for sc in straight s_curve turn encirclement; do
  echo ""
  echo "#################### 场景: $sc ####################"
  ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py \
      scenario:="$sc" time_scale:=0.0
done

echo ""
echo "全部场景运行完成。结果保存在: $PKG_SRC/results/"
