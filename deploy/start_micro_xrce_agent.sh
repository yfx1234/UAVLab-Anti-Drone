#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BRIDGE_SETUP="${BRIDGE_SETUP:-/home/uavlab/px4_bridge_ws/install/setup.bash}"
AGENT_DEVICE="${AGENT_DEVICE:-/dev/ttyTHS1}"
AGENT_BAUD="${AGENT_BAUD:-921600}"
AGENT_VERBOSITY="${AGENT_VERBOSITY:-3}"

if [[ ! -r "${ROS_SETUP}" ]]; then
  echo "missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -r "${BRIDGE_SETUP}" ]]; then
  echo "missing Micro XRCE-DDS Agent setup: ${BRIDGE_SETUP}" >&2
  exit 1
fi
if [[ ! -e "${AGENT_DEVICE}" ]]; then
  echo "missing serial device: ${AGENT_DEVICE}" >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${BRIDGE_SETUP}"
set -u

exec MicroXRCEAgent serial \
  --dev "${AGENT_DEVICE}" \
  -b "${AGENT_BAUD}" \
  -v "${AGENT_VERBOSITY}"
