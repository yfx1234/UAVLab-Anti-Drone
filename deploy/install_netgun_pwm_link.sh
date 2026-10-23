#!/usr/bin/env bash
# 安装网枪实物击发链路（2026-10-23；用户级 systemd，随登录会话开机自启；可重复执行）：
#   1. qgc-mavlink-forward.service 的 MAVProxy 增加本机输出 --out=udp:127.0.0.1:14555
#      （改前备份；有改动才重启转发服务——QGC 断开几秒，飞控与 uXRCE agent 不受影响）
#   2. 安装并（重）启 netgun-pwm-link.service（deploy/netgun_pwm_link.py serve）
# 不改飞控参数、不发任何击发指令；最后打印链路状态（飞控心跳 + AUX5 参数只读核对）。
# 拦截链运行中重跑会让击发服务重启约 1 s（期间开火的击发请求失败），先停链。
set -euo pipefail

INTERCEPTOR_WS="${INTERCEPTOR_WS:-$HOME/interceptor_ws}"
UNIT_DIR="$HOME/.config/systemd/user"
FORWARD_UNIT="$UNIT_DIR/qgc-mavlink-forward.service"
PWM_UNIT_SRC="$INTERCEPTOR_WS/deploy/systemd/netgun-pwm-link.service"
PWM_TOOL="$INTERCEPTOR_WS/deploy/netgun_pwm_link.py"
LOCAL_OUT="--out=udp:127.0.0.1:14555"

if ! python3 -c 'import pymavlink' 2>/dev/null; then
  echo "错误：缺少 pymavlink（pip3 install --user pymavlink）" >&2
  exit 1
fi
for f in "$PWM_UNIT_SRC" "$PWM_TOOL" "$FORWARD_UNIT"; do
  if [ ! -f "$f" ]; then
    echo "错误：缺少 $f" >&2
    exit 1
  fi
done
if [ "$(grep -c '^ExecStart=' "$FORWARD_UNIT")" != 1 ] || grep -q '^ExecStart=.*\\$' "$FORWARD_UNIT"; then
  echo "错误：$FORWARD_UNIT 的 ExecStart 不是单独一行，未修改；手动加 $LOCAL_OUT" >&2
  exit 1
fi

forward_changed=0
if ! grep -q -- "$LOCAL_OUT" "$FORWARD_UNIT"; then
  backup="$FORWARD_UNIT.bak_$(date +%Y%m%d_%H%M%S)"
  cp -p "$FORWARD_UNIT" "$backup"
  sed -i "/^ExecStart=/ s|\$| $LOCAL_OUT|" "$FORWARD_UNIT"
  forward_changed=1
  echo "转发服务加本机输出 $LOCAL_OUT（原文件备份 $backup）"
fi
install -m 0644 "$PWM_UNIT_SRC" "$UNIT_DIR/netgun-pwm-link.service"
systemctl --user daemon-reload
if [ "$forward_changed" = 1 ]; then
  systemctl --user restart qgc-mavlink-forward.service
  echo "已重启 qgc-mavlink-forward.service（QGC 断开几秒后自动重连）"
fi
systemctl --user enable netgun-pwm-link.service >/dev/null 2>&1
systemctl --user restart netgun-pwm-link.service
# 等飞控心跳与参数核对（通常 2–3 s）
for _ in $(seq 1 30); do
  if python3 "$PWM_TOOL" status >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
python3 "$PWM_TOOL" status --brief || true
