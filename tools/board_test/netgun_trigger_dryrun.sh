#!/usr/bin/env bash
# 实物击发钩子的干跑替身（I-3，2026-10-21）：只记录调用、不碰任何硬件。
# fullchain_test.py --trigger-hook 把它装订为 strike 的 trigger_command，
# 验证"开火恰执行一次 + 环境变量契约 + 异步不阻塞火控环"。
# 契约（真实 PWM 脚本照此实现）：/bin/sh -c 调用；环境变量
#   NETGUN_SHOT_ID      单发序号（恒为 1）
#   NETGUN_SHOT_REASON  PRIMARY_WINDOW / WINDOW_HOLD / MANUAL / BLIND_SHOT_*
# 退出码 0 = 击发成功（strike 记 "trigger command done rc=0"），非 0 记 FAILED。
set -eu
log="${NETGUN_TRIGGER_LOG:-/tmp/netgun_trigger_dryrun.log}"
printf '%s shot_id=%s reason=%s\n' "$(date +%s.%N)" \
  "${NETGUN_SHOT_ID:-?}" "${NETGUN_SHOT_REASON:-?}" >> "$log"
sleep 0.3  # 模拟 PWM 脉冲时长
