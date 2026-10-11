#!/bin/bash
# M3 完整闭环仿真:mock感知 -> 滤波 -> track_hold -> 安全阀门(gate+adapter live)
#   -> mock机体(飞控模拟)-> 状态回环。
# 场景A hover:目标悬停于正上方 10 m,35 s。
# 场景B circle:目标绕 (0,0,-8) 半径 3 m 转圈,45 s。
source /opt/ros/humble/setup.bash
source ~/Jetson_ROS2_V0/ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
T=~/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/tools
BAGROOT=/tmp/zed_bags

pkill -f "[m]ock_vehicle_node" 2>/dev/null
pkill -f "[m]ock_perception_node" 2>/dev/null
pkill -f "[z]ed_track_filter_node" 2>/dev/null
pkill -f "[t]rack_hold_node" 2>/dev/null
pkill -f "[u]pstream_gate_node" 2>/dev/null
pkill -f "[p]x4_upstream_adapter" 2>/dev/null
sleep 2

run_scenario() {
  local NAME=$1 MOTION=$2 CX=$3 CY=$4 CZ=$5 RADIUS=$6 RATE=$7 DURATION=$8
  local BAG=$BAGROOT/sim_$NAME
  rm -rf "$BAG"
  python3 $T/mock_vehicle_node.py > /tmp/sim_vehicle.log 2>&1 &
  local V=$!
  python3 $T/mock_perception_node.py --ros-args -p motion:=$MOTION \
    -p "center_ned:=[$CX,$CY,$CZ]" -p radius_m:=$RADIUS -p angular_rate:=$RATE \
    -p dropout_rate:=0.08 > /tmp/sim_perception.log 2>&1 &
  local M=$!
  ros2 run zed_tracking zed_track_filter_node.py > /tmp/sim_filter.log 2>&1 &
  local F=$!
  ros2 run zed_tracking track_hold_node.py > /tmp/sim_hold.log 2>&1 &
  local H=$!
  ros2 run falcon_intercept upstream_gate_node > /tmp/sim_gate.log 2>&1 &
  local G=$!
  ros2 run falcon_intercept px4_upstream_adapter_node --ros-args \
    -p mode:=live -p allow_live_control:=true -p calibration_ready:=true \
    > /tmp/sim_adapter.log 2>&1 &
  local A=$!
  python3 $T/shot_event_bridge.py --ros-args -p truth_source:=sim \
    > /tmp/sim_bridge.log 2>&1 &
  local B=$!
  ros2 launch virtual_interception_referee referee.launch.py \
    pursuer_local_topic:=/fmu/out/vehicle_local_position \
    pursuer_attitude_topic:=/fmu/out/vehicle_attitude \
    attack_axis_body:=body_x_positive \
    > /tmp/sim_referee.log 2>&1 &
  local R=$!
  sleep 6
  timeout $DURATION ros2 bag record -o "$BAG" \
    /sim/vehicle_state /sim/target_truth /zed_tracking/track_hold_status \
    /zed_tracking/shot_event /zed_tracking/track_json \
    /zed_perception_node/target_json /evaluation/virtual_hit_result \
    /evaluation/virtual_solver_hit_result \
    > /dev/null 2>&1
  echo "=== 场景 $NAME:track_hold 日志尾部 ==="
  tail -5 /tmp/sim_hold.log
  echo "=== 场景 $NAME:adapter 日志尾部 ==="
  grep -aE "GATE|ready" /tmp/sim_adapter.log | tail -3
  # kill $PID 杀不掉 ros2 run 的子进程,必须按模式清理,
  # 否则残留实例污染下一场景(2026-10-07 双实例事故)。
  kill $V $M $F $H $G $A $B $R 2>/dev/null
  sleep 1
  pkill -f "[m]ock_vehicle_node" 2>/dev/null
  pkill -f "[m]ock_perception_node" 2>/dev/null
  pkill -f "[z]ed_track_filter_node" 2>/dev/null
  pkill -f "[t]rack_hold_node" 2>/dev/null
  pkill -f "[u]pstream_gate_node" 2>/dev/null
  pkill -f "[p]x4_upstream_adapter" 2>/dev/null
  pkill -f "[s]hot_event_bridge" 2>/dev/null
  pkill -f "[v]irtual_projectile_referee" 2>/dev/null
  pkill -f "[r]eferee.launch" 2>/dev/null
  sleep 3
  echo "=== 场景 $NAME:分析 ==="
  python3 $T/sim_analyze.py "$BAG" $MOTION $CX $CY $CZ $RADIUS $RATE
}

run_scenario hover  hover  0.0 0.0 -10.0 0.0 0.0  35
run_scenario circle circle 0.0 0.0 -8.0  3.0 0.5  45
echo "SIM_ALL_DONE"
