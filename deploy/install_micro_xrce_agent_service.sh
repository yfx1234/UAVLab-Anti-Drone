#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/home/uavlab/Jetson_ROS2_V0}"
UNIT_SOURCE="${PROJECT_ROOT}/deploy/systemd/micro-xrce-agent.service"
UNIT_TARGET="/etc/systemd/system/micro-xrce-agent.service"

if [[ ! -f "${UNIT_SOURCE}" ]]; then
  echo "missing unit file: ${UNIT_SOURCE}" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_ROOT}/deploy/start_micro_xrce_agent.sh" ]]; then
  echo "starter is not executable: ${PROJECT_ROOT}/deploy/start_micro_xrce_agent.sh" >&2
  echo "run: chmod +x ${PROJECT_ROOT}/deploy/start_micro_xrce_agent.sh" >&2
  exit 1
fi

if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo "a manually started MicroXRCEAgent is already running" >&2
  echo "stop it before installing the systemd-managed instance" >&2
  exit 1
fi

sudo install -o root -g root -m 0644 "${UNIT_SOURCE}" "${UNIT_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service

sudo systemctl --no-pager --full status micro-xrce-agent.service
echo
echo "Service installed. Reboot once and verify it again before flight."
