#!/usr/bin/env bash
# 实物调试改造验证：A) 实物 launch 参数冒烟 B) 仿真回归+调试话题采样

source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash

echo "=== A) 实物 launch 参数冒烟（8s，节点应存活无参数错误）==="
timeout 8 ros2 launch netgun_intercept netgun_real.launch.py > /tmp/real_smoke.log 2>&1
grep -acE "error|Error|exception|died" /tmp/real_smoke.log || true
grep -aE "process started" /tmp/real_smoke.log | wc -l
timeout 3 ros2 topic echo --once /debug/estimator_state 2>/dev/null | head -2 || echo "(estimator debug: 采样于冒烟已结束，跳过)"

echo "=== B) 仿真回归（pursuit@1）+ 调试话题采样 ==="
OUT=~/netgun_logs/debug_check
mkdir -p $OUT
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
  > $OUT/sim.log 2>&1 &
LAUNCH=$!
sleep 45   # 走过 TRACK→授权→AIM 窗口
echo "--- /debug/guidance_state ---"
timeout 3 ros2 topic echo --once /debug/guidance_state 2>/dev/null | head -2
echo "--- /debug/strike_state ---"
timeout 3 ros2 topic echo --once /debug/strike_state 2>/dev/null | head -2
echo "--- /debug/estimator_state ---"
timeout 3 ros2 topic echo --once /debug/estimator_state 2>/dev/null | head -2
wait $LAUNCH
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -x MicroXRCEAgent
echo "=== 关键事件 ==="
grep -aE "→ AIM|FIRE|REFEREE|spin done|settled" $OUT/sim.log | head -6
