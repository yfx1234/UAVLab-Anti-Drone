#!/bin/bash
# board_flight_status.sh:飞行状态聚合输出(单次执行,输出 JSON)。
# 面板通过 SSH 调用;数据源为 flight_status_watcher 落地文件 + 进程检查。
source /opt/ros/humble/setup.bash 2>/dev/null
WS=/home/uavlab/Jetson_ROS2_V0/ros2_ws
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

python3 - << 'PYEOF'
import json, subprocess, time, os

def running(pattern):
    result = subprocess.run(
        ["pgrep", "-f", pattern], capture_output=True, text=True)
    return bool(result.stdout.strip())

def read_status():
    try:
        with open("/tmp/flight_status.json", encoding="utf-8") as handle:
            payload = json.load(handle)
        if time.time() - payload.get("written_s", 0) > 5:
            return {"stale": True}
        return payload
    except (OSError, ValueError):
        return None

def read_first(path):
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return None

def thermal():
    # Orin Nano 的 zone 名为 cpu-thermal / gpu-thermal(小写)。
    readings = []
    try:
        zones = sorted(
            (name for name in os.listdir("/sys/class/thermal")
             if name.startswith("thermal_zone")))
        for zone in zones:
            try:
                with open(f"/sys/class/thermal/{zone}/type", encoding="utf-8") as h:
                    kind = h.read().strip().lower()
                if kind in ("cpu-thermal", "gpu-thermal"):
                    with open(f"/sys/class/thermal/{zone}/temp", encoding="utf-8") as h:
                        readings.append((kind, int(h.read().strip()) / 1000.0))
            except OSError:
                continue
    except OSError:
        pass
    if not readings:
        return None
    return {"cpu_c": max(v for k, v in readings if k == "cpu-thermal") if
            any(k == "cpu-thermal" for k, v in readings) else None,
            "gpu_c": max(v for k, v in readings if k == "gpu-thermal") if
            any(k == "gpu-thermal" for k, v in readings) else None}

payload = {
    "stamp_s": time.time(),
    "watcher_running": running("flight_status_watcher"),
    "watcher": read_status(),
    "processes": {
        "perception": running("zed_perception_node"),
        "filter": running("zed_track_filter_node"),
        "control": running("track_hold_node"),
        "adapter": running("px4_upstream_adapter"),
        "referee": running("virtual_projectile_referee"),
        "preview": running("zed_preview"),
    },
    "chain_mode": read_first("/tmp/falcon_zed_flight/mode"),
    "thermal_c": thermal(),
}
print(json.dumps(payload, ensure_ascii=False))
PYEOF
