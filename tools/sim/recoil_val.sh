#!/usr/bin/env bash
# 后坐扰动验证运行器：straight@3 + 冲量注入 + bag 记录 z(t) 复盘
OUT=~/netgun_logs/recoil_val
BAG=$OUT/bag
mkdir -p $OUT
rm -rf $BAG
pkill -f "bin/px4" 2>/dev/null; sleep 2
pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
export GZ_SIM_PLUGIN_PATH=/home/key/netgun_plugins
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export PURSUER_POSE="0,-2,0.01"
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT=$!
sleep 2
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
if [ "$R" != "1" ]; then echo BOOT_FAIL > $OUT/status.txt; pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $AGENT; exit 1; fi
sleep 8
ros2 bag record -o $BAG /px4_1/fmu/out/vehicle_local_position > /tmp/bagrec2.log 2>&1 &
BAGPID=$!
sleep 2
LOG=$OUT/straight3.log
timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=3.0 target_radius_m:=20.0 \
  target_initial_north_m:=0.0 yaw_course_offset_rad:=0.0 depth_valid_max_m:=-1.0 \
  > $LOG 2>&1 &
LAUNCH_PID=$!
# watcher 后台等 FIRE 并注入冲量
bash ~/recoil_watch.sh $LOG 5.0 0.1 &
WATCH_PID=$!
wait $LAUNCH_PID
sleep 2
kill -INT $BAGPID 2>/dev/null; sleep 3; kill -9 $BAGPID 2>/dev/null
kill $WATCH_PID 2>/dev/null
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
echo DONE > $OUT/status.txt
grep -E "FIRE|REFEREE RESULT" $LOG | head -3 > $OUT/summary.txt
cat $OUT/summary.txt
