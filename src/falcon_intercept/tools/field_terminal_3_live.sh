#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/field_terminal_common.sh"
exec ros2 launch falcon_intercept field_terminal_3_live.launch.py field_authorized:=true
