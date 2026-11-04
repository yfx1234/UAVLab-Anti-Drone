#!/usr/bin/env bash
# 舵机 MAVLink 直控服务安装（通用工具，独立于拦截链）。
# 用法：bash install_servo_mavlink.sh
# 动作：
#   1. 安装用户级 systemd 服务 servo-mavlink（常驻 servo_mavlink.py serve）
#   2. 检查/补齐 MAVProxy 转发（qgc-mavlink-forward.service）的
#      --out=udp:127.0.0.1:14560 输出口（舵机链路依赖）；补齐会重启该服务，
#      QGC 链路约 9 秒自动恢复（监督器在）
#   3. 启动并使能 servo-mavlink
set -euo pipefail

SCRIPT_PATH="$HOME/interceptor_ws/deploy/servo_mavlink.py"
UNIT_DIR="$HOME/.config/systemd/user"
UNIT="$UNIT_DIR/servo-mavlink.service"
MAVPROXY_UNIT="$UNIT_DIR/qgc-mavlink-forward.service"
OUT_ARG="--out=udp:127.0.0.1:14560"

if [ ! -f "$SCRIPT_PATH" ]; then
  echo "错误：未找到 $SCRIPT_PATH" >&2
  exit 1
fi

# ---- 1. 服务单元 ----
mkdir -p "$UNIT_DIR"
cat > "$UNIT" <<UNIT_EOF
[Unit]
Description=Servo MAVLink direct control (via MAVProxy)
[Service]
ExecStart=/usr/bin/python3 -u %h/interceptor_ws/deploy/servo_mavlink.py serve
Restart=on-failure
RestartSec=2
[Install]
WantedBy=default.target
UNIT_EOF
systemctl --user daemon-reload

# ---- 2. MAVProxy 输出口 ----
if [ ! -f "$MAVPROXY_UNIT" ]; then
  echo "警告：未找到 $MAVPROXY_UNIT，跳过输出口检查（若舵机命令无 ACK，请手动加 $OUT_ARG）" >&2
elif grep -q -- "$OUT_ARG" "$MAVPROXY_UNIT"; then
  echo "MAVProxy 输出口已存在：$OUT_ARG"
else
  cp "$MAVPROXY_UNIT" "$MAVPROXY_UNIT.bak_servo_$(date +%s)"
  sed -i "s|^\(ExecStart=.*\)$|\1 $OUT_ARG|" "$MAVPROXY_UNIT"
  systemctl --user daemon-reload
  systemctl --user restart qgc-mavlink-forward
  echo "MAVProxy 已补 $OUT_ARG 并重启（QGC 链约 9 秒自动恢复）"
fi

# ---- 3. 启动舵机服务 ----
systemctl --user enable --now servo-mavlink
sleep 2
if [ "$(systemctl --user is-active servo-mavlink)" = "active" ]; then
  echo "servo-mavlink 已启动（控制口 ~/servo_panel.sock）"
  echo "验证：python3 ~/interceptor_ws/deploy/servo_mavlink.py state"
else
  echo "错误：servo-mavlink 未启动，查看：journalctl --user -u servo-mavlink -n 20" >&2
  exit 1
fi
