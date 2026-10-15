#!/usr/bin/env bash
# Offline bearing-only replay test v2: clean process management + windowed metrics
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
OUT=~/netgun_logs/yaw_ab
pkill -x relative_estimator 2>/dev/null; sleep 2

start_estimator() {
  ros2 run px4_offboard_tracking relative_estimator --ros-args \
    -p observation_topic:=/replay/obs \
    -p attitude_topic:=/replay/att \
    -p estimate_topic:=/replay/est \
    -p camera_fx:=1274.0 -p camera_fy:=1274.0 -p camera_cx:=960.0 -p camera_cy:=540.0 \
    -p camera_width:=1920 -p camera_height:=1080 \
    -p camera_q_body_to_optical_w:=0.0 -p camera_q_body_to_optical_x:=1.0 \
    -p camera_q_body_to_optical_y:=0.0 -p camera_q_body_to_optical_z:=0.0 \
    -p attitude_sync_tolerance_s:=0.2 \
    -p innovation_gate_mahalanobis:=25.0 \
    -p acceleration_noise_stddev_mps2:=4.0 \
    > $OUT/est_$1.log 2>&1 &
}

for MODE in baseline:-999 strip8:8 stripall:0.0001; do
  NAME="${MODE%%:*}"; TH="${MODE##*:}"
  pkill -x relative_estimator 2>/dev/null; sleep 1
  start_estimator $NAME
  sleep 3
  python3 ~/replay_bearing.py $OUT/bag_v2a $TH $OUT/replay_$NAME.json > $OUT/replay_$NAME.out 2>&1
  pkill -x relative_estimator 2>/dev/null
  sleep 1
  echo "=== $NAME (strip>$TH m) ==="
  cat $OUT/replay_$NAME.out | grep -v rosbag2
  grep -o "bo=[0-9]*" $OUT/est_$NAME.log | tail -1
done
echo REPLAY_ALL_DONE
