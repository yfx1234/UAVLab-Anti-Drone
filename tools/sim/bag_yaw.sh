#!/usr/bin/env bash
# Bag-instrumented yaw servo run: record attitude/local_position/truth/obs/est
OUT=~/netgun_logs/yaw_ab
BAG=$OUT/bag_v2a
mkdir -p $OUT
rm -rf $BAG
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
if [ "$R" != "1" ]; then echo "BOOT_FAIL" > $OUT/bag_v2a_status.txt; pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $AGENT 2>/dev/null; exit 1; fi
sleep 8
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1
ros2 bag record -o $BAG \
  /px4_1/fmu/out/vehicle_attitude \
  /px4_1/fmu/out/vehicle_local_position \
  /evaluation/relative_truth \
  /perception/depth/observation \
  /tracking/relative/estimate \
  /tracking/virtual_shot_event > /tmp/bagrec.log 2>&1 &
BAGPID=$!
sleep 3
timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=5.0 target_radius_m:=20.0 \
  target_initial_north_m:=0.0 yaw_course_offset_rad:=0.0 \
  > $OUT/bag_v2a.log 2>&1
sleep 2
kill -INT $BAGPID 2>/dev/null; sleep 3; kill -9 $BAGPID 2>/dev/null
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
echo DONE > $OUT/bag_v2a_status.txt
grep -E "FIRE|REFEREE RESULT" $OUT/bag_v2a.log | head -4 > $OUT/bag_v2a_summary.txt
cat $OUT/bag_v2a_summary.txt
