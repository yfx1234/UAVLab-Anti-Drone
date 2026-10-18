#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BRIDGE_SETUP="${BRIDGE_SETUP:-/home/uavlab/px4_bridge_ws/install/setup.bash}"
PROJECT_SETUP="${PROJECT_SETUP:-/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash}"

for setup_file in "${ROS_SETUP}" "${BRIDGE_SETUP}"; do
  if [[ ! -r "${setup_file}" ]]; then
    echo "missing ROS setup: ${setup_file}" >&2
    exit 1
  fi
done

source "${ROS_SETUP}"
source "${BRIDGE_SETUP}"
if [[ -r "${PROJECT_SETUP}" ]]; then
  source "${PROJECT_SETUP}"
fi

# F-5（2026-10-18）：发现配置单一真源=deploy/px4_ros2_environment.sh
# （ROS_LOCALHOST_ONLY=0，与拦截链一致）。daemon 拿图信息用这套环境，
# localhost-only 会让 ros2 topic list/info/echo 看不到拦截链话题。
ENV_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/px4_ros2_environment.sh"
if [[ ! -r "${ENV_FILE}" ]]; then
  echo "missing environment file: ${ENV_FILE}" >&2
  exit 1
fi
source "${ENV_FILE}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Remove a daemon inherited from an older discovery environment, then create
# the single boot-managed daemon with the flight baseline.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start
