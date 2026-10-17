#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/home/uavlab/Jetson_ROS2_V0}"
AGENT_UNIT_SOURCE="${PROJECT_ROOT}/deploy/systemd/micro-xrce-agent.service"
DAEMON_UNIT_SOURCE="${PROJECT_ROOT}/deploy/systemd/ros2-px4-daemon.service"
ENV_SOURCE="${PROJECT_ROOT}/deploy/px4_ros2_environment.sh"
AGENT_UNIT_TARGET="/etc/systemd/system/micro-xrce-agent.service"
DAEMON_UNIT_TARGET="/etc/systemd/system/ros2-px4-daemon.service"
PROFILE_TARGET="/etc/profile.d/px4-ros2.sh"
USER_BASHRC="/home/uavlab/.bashrc"
SOURCE_LINE="source /etc/profile.d/px4-ros2.sh"

for required_file in \
  "${AGENT_UNIT_SOURCE}" \
  "${DAEMON_UNIT_SOURCE}" \
  "${ENV_SOURCE}" \
  "${PROJECT_ROOT}/deploy/start_micro_xrce_agent.sh" \
  "${PROJECT_ROOT}/deploy/start_ros2_px4_daemon.sh" \
  "${PROJECT_ROOT}/deploy/stop_ros2_px4_daemon.sh"; do
  if [[ ! -f "${required_file}" ]]; then
    echo "missing deployment file: ${required_file}" >&2
    exit 1
  fi
done

sudo install -o root -g root -m 0644 "${AGENT_UNIT_SOURCE}" "${AGENT_UNIT_TARGET}"
sudo install -o root -g root -m 0644 "${DAEMON_UNIT_SOURCE}" "${DAEMON_UNIT_TARGET}"
sudo install -o root -g root -m 0644 "${ENV_SOURCE}" "${PROFILE_TARGET}"

if ! grep -Fqx "${SOURCE_LINE}" "${USER_BASHRC}"; then
  cp "${USER_BASHRC}" "${USER_BASHRC}.before-px4-ros2-runtime"
  printf '\n# PX4 ROS 2 flight discovery baseline\n%s\n' "${SOURCE_LINE}" >>"${USER_BASHRC}"
fi

sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
sudo systemctl restart ros2-px4-daemon.service
sudo systemctl enable ros2-px4-daemon.service

echo "agent_enabled=$(systemctl is-enabled micro-xrce-agent.service)"
echo "agent_active=$(systemctl is-active micro-xrce-agent.service)"
echo "daemon_enabled=$(systemctl is-enabled ros2-px4-daemon.service)"
echo "daemon_active=$(systemctl is-active ros2-px4-daemon.service)"

# ROS Humble setup scripts read optional variables that may be unset. Temporarily
# disable nounset while sourcing them, as done by the Agent starter.
set +u
source "${ENV_SOURCE}"
source /opt/ros/humble/setup.bash
source /home/uavlab/px4_bridge_ws/install/setup.bash
set -u
sleep 3
echo "fmu_topic_count=$(ros2 topic list | grep -c '^/fmu/' || true)"
