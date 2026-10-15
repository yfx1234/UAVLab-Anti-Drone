#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-$HOME/Jetson_ROS2_V0/ros2_ws}"
SOLVER_PID=""
cleanup() {
  [[ -z "$SOLVER_PID" ]] || kill -- "-$SOLVER_PID" 2>/dev/null || true
  [[ -z "$SOLVER_PID" ]] || wait "$SOLVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

setsid timeout 15s ros2 run falcon_intercept stereo_target_solver_node --ros-args \
  -p fx:=320.0 -p fy:=320.0 -p cx:=320.0 -p cy:=240.0 \
  -p baseline_m:=0.2 \
  -p horizontal_zero_disparity_offset_px:=0.0 \
  -p horizontal_disparity_sign:=1.0 \
  >/tmp/falcon_target_camera_position_solver.log 2>&1 &
SOLVER_PID=$!
sleep 1
ros2 topic info /perception/target/camera_position --verbose \
  >/tmp/falcon_target_camera_position_qos.log
grep -q 'Reliability: BEST_EFFORT' /tmp/falcon_target_camera_position_qos.log
PYTHONPATH="src/falcon_intercept:${PYTHONPATH:-}" python3 \
  src/falcon_intercept/tools/target_camera_position_contract_smoke.py
echo "TARGET_CAMERA_POSITION_QOS_PASS reliability=BEST_EFFORT depth=1_configured"
tail -n 50 /tmp/falcon_target_camera_position_solver.log
