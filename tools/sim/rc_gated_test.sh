#!/usr/bin/env bash
# rc_gated 门控验证：假飞手流程 = 待机(验证零setpoint) → 解锁 → 切offboard
# (验证起流+飞行) → 切回posctl(验证停流+standby日志)
OUT=~/netgun_logs/rc_gated_test
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
LOG=$OUT/rcg.log
timeout 100 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=3.0 target_radius_m:=20.0 \
  target_initial_north_m:=0.0 engage_mode:=rc_gated \
  > $LOG 2>&1 &
LAUNCH_PID=$!
SP=/px4_1/fmu/in/trajectory_setpoint
CMD=/px4_1/fmu/in/vehicle_command
sleep 30
N1=$(timeout 3 ros2 topic echo $SP 2>/dev/null | head -3 | wc -l)
echo "CHECK1_pre-arm_setpoint_msgs=${N1}  (expect 0)"
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 1.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" > /dev/null 2>&1
sleep 2
N2=$(timeout 3 ros2 topic echo $SP 2>/dev/null | head -3 | wc -l)
echo "CHECK2_armed_not_offboard_setpoint_msgs=${N2}  (expect 0)"
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 176, param1: 1.0, param2: 6.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" > /dev/null 2>&1
sleep 8
N3=$(timeout 3 ros2 topic echo $SP 2>/dev/null | head -3 | wc -l)
echo "CHECK3_after_offboard_switch_setpoint_msgs=${N3}  (expect >0)"
sleep 20
ros2 topic pub --once $CMD px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 176, param1: 1.0, param2: 3.0, target_system: 2, target_component: 1, source_system: 255, source_component: 1, from_external: true}" > /dev/null 2>&1
sleep 4
N4=$(timeout 3 ros2 topic echo $SP 2>/dev/null | head -3 | wc -l)
echo "CHECK4_after_exit_setpoint_msgs=${N4}  (expect 0)"
wait $LAUNCH_PID 2>/dev/null
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
echo ===GUIDANCE_EVENTS===
grep -E "task origin|phase →|offboard exited|yaw servo|track →" $LOG | head -10
