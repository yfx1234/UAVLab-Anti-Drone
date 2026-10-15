#!/usr/bin/env bash
# 诊断：调试话题是否存在 + daemon 处置后重采
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
pkill -f "bin/px4" 2>/dev/null; sleep 2
pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -x MicroXRCEAgent 2>/dev/null; sleep 1
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export PURSUER_POSE="0,-2,0.01"
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT=$!
sleep 2
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
[ "$R" = "1" ] || { echo BOOT_FAIL; kill $AGENT; exit 1; }
sleep 8
ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=1.0 target_radius_m:=20.0 \
  > /tmp/dbg_sim2.log 2>&1 &
LAUNCH=$!
sleep 20
echo "=== topic list (debug) ==="
timeout 5 ros2 topic list 2>/dev/null | grep -a debug
echo "=== node log check: guidance/strike/estimator alive? ==="
grep -ac "guidance ready" /tmp/dbg_sim2.log
echo "=== echo try 1 ==="
timeout 4 ros2 topic echo --once /debug/guidance_state 2>&1 | head -3
echo "=== daemon restart ==="
ros2 daemon stop; sleep 1; ros2 daemon start; sleep 3
echo "=== echo try 2 ==="
timeout 4 ros2 topic echo --once /debug/guidance_state 2>&1 | head -3
timeout 4 ros2 topic echo --once /debug/strike_state 2>&1 | head -3
timeout 4 ros2 topic echo --once /debug/estimator_state 2>&1 | head -3
kill $LAUNCH 2>/dev/null; sleep 2
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -x MicroXRCEAgent
echo done
