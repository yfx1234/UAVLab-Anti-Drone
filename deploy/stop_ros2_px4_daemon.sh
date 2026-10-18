#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
# F-5（2026-10-18）：与 start 脚本同源 px4_ros2_environment.sh（=0）。
# ros2 daemon 必须在创建它的同一发现环境里才能被 stop 命中
ENV_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/px4_ros2_environment.sh"
if [[ -r "${ENV_FILE}" ]]; then
  source "${ENV_FILE}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

ros2 daemon stop >/dev/null 2>&1 || true
