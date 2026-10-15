#!/usr/bin/env bash
# AIM 冒烟：单场景全链运行（结构对齐 run_matrix_geo2.sh）
# 用法: run_aim_smoke.sh <tag> <scenario> <speed> <north> <radius> [extra launch args...]
TAG=$1; SC=$2; SP=$3; NORTH=$4; RADIUS=$5; shift 5
OUT=~/netgun_logs/aim_smoke
mkdir -p $OUT
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
r=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { r=1; break; }; sleep 1; done
LOG=$OUT/${TAG}.log
if [ "$r" != "1" ]; then echo "BOOT_FAIL"; kill $AGENT 2>/dev/null; pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; exit 1; fi
sleep 8
timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=$SC target_speed_mps:=$SP target_radius_m:=$RADIUS \
  target_initial_north_m:=$NORTH \
  yaw_course_offset_rad:=0.0 depth_valid_max_m:=-1.0 \
  "$@" \
  > $LOG 2>&1
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
echo "=== $TAG key events ==="
grep -aE "stable track \+ weapon auth|aim timeout|aim est lost|NETGUN FIRE|shot event|spin done|REFEREE RESULT" $LOG
echo "=== $TAG fire window diag ==="
grep -a "netgun diag" $LOG | tail -8
