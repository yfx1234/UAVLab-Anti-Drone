#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
cd "${WORKSPACE}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
