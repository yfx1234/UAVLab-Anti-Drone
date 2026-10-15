#!/usr/bin/env bash
# 隔离测试：起双机仿真（不起 ROS 链），向追踪机发 500N/1s 冲量，
# 从 PX4 日志判断物理是否生效（解锁待机态飞机应被砸翻/触发保护）
OUT=~/netgun_logs/recoil_manual
mkdir -p $OUT
pkill -f "bin/px4" 2>/dev/null; sleep 2
pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
export GZ_SIM_PLUGIN_PATH=$HOME/netgun_plugins
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export PURSUER_POSE="0,-2,0.01"
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT=$!
sleep 2
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
[ "$R" = "1" ] || { echo BOOT_FAIL; kill $AGENT; exit 1; }
sleep 10   # 等模型稳定
echo "publishing 500N for 1s (plugin duration is 0.1s x10? no - single msg)"
# 插件 duration 由 SDF 固定 0.1s；连发 10 次覆盖 1s
for i in $(seq 1 10); do
  gz topic -t /netgun/recoil_cmd -m gz.msgs.EntityWrench \
    -p 'entity: {name: "pursuer"} wrench: {force: {x: 300 y: 0 z: 0}}'
  sleep 0.1
done
sleep 4
L=$(ls -td /home/key/px4sim/logs/dual/*/ | head -1)
echo "=== plugin diag ==="
grep -E "RecoilPlugin" $L/gazebo.log | tail -5
echo "=== px4 pursuer tail ==="
tail -6 $L/px4_1_pursuer.log | cut -c1-110
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
