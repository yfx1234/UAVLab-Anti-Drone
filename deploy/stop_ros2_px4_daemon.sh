#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

ros2 daemon stop >/dev/null 2>&1 || true
