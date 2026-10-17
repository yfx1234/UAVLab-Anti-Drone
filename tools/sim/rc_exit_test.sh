#!/usr/bin/env bash
# rc_gated 退出路径专项：进入 offboard 飞行后切 POSCTL，
# 回读 nav_state + 验证 standby 日志与 setpoint 停流
OUT=~/netgun_logs/rc_exit_test
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
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
[ "$R" = "1" ] || { echo BOOT_FAIL; kill $AGENT; exit 1; }
sleep 8
LOG=$OUT/rcx.log
timeout 95 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=1.0 target_radius_m:=20.0 \
  target_initial_north_m:=0.0 engage_mode:=rc_gated \
  > $LOG 2>&1 &
LAUNCH_PID=$!
CMD=/px4_1/fmu/in/vehicle_command
VS=/px4_1/fmu/out/vehicle_status
sleep 30
echo "--- pilot: ARM"
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 1.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" 2>&1 | tail -1
sleep 2
echo "--- pilot: OFFBOARD"
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 176, param1: 1.0, param2: 6.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" 2>&1 | tail -1
sleep 15   # 爬升进 TRACK
echo "--- nav_state before exit:"
timeout 2 ros2 topic echo $VS --field nav_state 2>/dev/null | head -1
echo "--- pilot: POSCTL (exit offboard)"
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 176, param1: 1.0, param2: 3.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" 2>&1 | tail -1
sleep 3
echo "--- nav_state after exit:"
timeout 2 ros2 topic echo $VS --field nav_state 2>/dev/null | head -1
sleep 2
wait $LAUNCH_PID 2>/dev/null
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
echo "=== guidance events ==="
grep -E "phase →|offboard exited|track →" $LOG | tail -6
echo "=== standby count (expect >=1) ==="
grep -c "offboard exited" $LOG
