#!/usr/bin/env bash
# 后坐/发射扰动 watcher v3：FIRE 后向 /netgun/recoil_cmd 发 EntityWrench
# （由世界内 RecoilPlugin 写入 ExternalWorldWrenchCmd 组件施加）
LOG=$1
IMPULSE=${2:-5.0}
DUR=${3:-0.1}
OUTLOG=~/netgun_logs/recoil_applied.txt
while ! grep -q "NETGUN FIRE" "$LOG" 2>/dev/null; do sleep 0.2; done
F=$(python3 -c "print($IMPULSE/$DUR)")
LAT=$(python3 -c "import random; print(round((random.random()-0.5)*0.4*$F,1))")
echo "$(date +%T) FIRE -> publish Fz=-$F N x=$LAT (plugin holds ${DUR}s)" >> $OUTLOG
gz topic -t /netgun/recoil_cmd -m gz.msgs.EntityWrench \
  -p "entity: {name: \"pursuer\"} wrench: {force: {x: $LAT y: 0 z: -$F}}" \
  >> $OUTLOG 2>&1
echo "$(date +%T) publish done" >> $OUTLOG
