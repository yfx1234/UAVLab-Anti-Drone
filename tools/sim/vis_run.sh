#!/usr/bin/env bash
# 轨迹可视化数据采集：用法 vis_run.sh <tag> <scenario> <speed> <north> <radius>
TAG=$1; SC=$2; SP=$3; NORTH=$4; RADIUS=$5
VIS=~/netgun_logs/vis
mkdir -p $VIS
rm -rf $VIS/$TAG
pkill -f "bin/px4" 2>/dev/null; sleep 2
pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export PURSUER_POSE="0,-2,0.01"
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT=$!
sleep 2
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
if [ "$R" != "1" ]; then
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $AGENT 2>/dev/null
  echo BOOT_FAIL; exit 1
fi
sleep 8
ros2 bag record -o $VIS/$TAG \
  /px4_1/fmu/out/vehicle_local_position \
  /px4_1/fmu/out/vehicle_attitude \
  /evaluation/relative_truth > /tmp/bagrec_v.log 2>&1 &
BAGPID=$!
sleep 3
timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=$SC target_speed_mps:=$SP target_radius_m:=$RADIUS \
  target_initial_north_m:=$NORTH yaw_course_offset_rad:=0.0 depth_valid_max_m:=-1.0 \
  > $VIS/$TAG.log 2>&1
sleep 2
kill -INT $BAGPID 2>/dev/null; sleep 3; kill -9 $BAGPID 2>/dev/null
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
grep -E "FIRE|REFEREE RESULT" $VIS/$TAG.log | tail -2
