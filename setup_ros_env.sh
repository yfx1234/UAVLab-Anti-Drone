#!/usr/bin/env bash
# Source this file to enter the Jetson ROS 2 workspace environment:
#   source /home/uavlab/Jetson_ROS2_V0/setup_ros_env.sh

_jetson_ros2_root="/home/uavlab/Jetson_ROS2_V0"
_jetson_ros2_ws="${_jetson_ros2_root}/ros2_ws"

if [[ ! -d "${_jetson_ros2_ws}" ]]; then
  echo "Missing ROS 2 workspace: ${_jetson_ros2_ws}" >&2
  return 1 2>/dev/null || exit 1
fi

cd "${_jetson_ros2_ws}" || return 1 2>/dev/null || exit 1

_jetson_ros2_had_nounset=0
case $- in
  *u*) _jetson_ros2_had_nounset=1; set +u ;;
esac

source /opt/ros/humble/setup.bash
source "${_jetson_ros2_ws}/install/setup.bash"

if [[ "${_jetson_ros2_had_nounset}" -eq 1 ]]; then
  set -u
fi

export JETSON_ROS2_V0="${_jetson_ros2_root}"
export JETSON_ROS2_WS="${_jetson_ros2_ws}"

echo "Jetson ROS 2 environment ready: ${JETSON_ROS2_WS}"

unset _jetson_ros2_root
unset _jetson_ros2_ws
unset _jetson_ros2_had_nounset
