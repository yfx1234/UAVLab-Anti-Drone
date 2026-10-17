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

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# Remove a daemon inherited from an older discovery environment, then create
# the single boot-managed daemon with the flight baseline.
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start
