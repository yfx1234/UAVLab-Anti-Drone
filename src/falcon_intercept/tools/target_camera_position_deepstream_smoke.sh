#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-$HOME/Jetson_ROS2_V0/ros2_ws}"
CALIBRATION="${FALCON_STEREO_CALIBRATION:-$WORKSPACE/install/falcon_intercept/share/falcon_intercept/config/camera/stereo_calib.yaml}"
DEEPSTREAM_PID=""
SOLVER_PID=""
cleanup() {
  [[ -z "$DEEPSTREAM_PID" ]] || kill -- "-$DEEPSTREAM_PID" 2>/dev/null || true
  [[ -z "$SOLVER_PID" ]] || kill -- "-$SOLVER_PID" 2>/dev/null || true
  [[ -z "$DEEPSTREAM_PID" ]] || wait "$DEEPSTREAM_PID" 2>/dev/null || true
  [[ -z "$SOLVER_PID" ]] || wait "$SOLVER_PID" 2>/dev/null || true
}
trap cleanup EXIT

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export LD_LIBRARY_PATH="/opt/nvidia/deepstream/deepstream-7.1/lib:${LD_LIBRARY_PATH:-}"
export GST_PLUGIN_PATH="$WORKSPACE/install/falcon_intercept_deepstream/lib/gstreamer-1.0:${GST_PLUGIN_PATH:-}"

setsid timeout 45s ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=false batch_size:=1 >/tmp/falcon_camera_position_deepstream.log 2>&1 &
DEEPSTREAM_PID=$!
setsid timeout 45s ros2 run falcon_intercept stereo_target_solver_node --ros-args \
  -p calibration_path:="$CALIBRATION" \
  >/tmp/falcon_camera_position_solver.log 2>&1 &
SOLVER_PID=$!

for _ in $(seq 1 100); do
  if grep -q 'DeepStream native pipeline started' /tmp/falcon_camera_position_deepstream.log; then
    break
  fi
  sleep 0.2
done
grep -q 'DeepStream native pipeline started' /tmp/falcon_camera_position_deepstream.log
PYTHONPATH="src/falcon_intercept:${PYTHONPATH:-}" python3 \
  src/falcon_intercept/tools/target_camera_position_rate_smoke.py
echo "TARGET_CAMERA_POSITION_DEEPSTREAM_LOG_TAIL"
tail -n 40 /tmp/falcon_camera_position_deepstream.log
tail -n 20 /tmp/falcon_camera_position_solver.log
