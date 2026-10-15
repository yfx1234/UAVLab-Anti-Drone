#!/usr/bin/env bash
# circle@4 失败轮诊断
for f in ~/netgun_logs/matrix_aim3/r*pursuit_circle_4.0.log ~/netgun_logs/matrix_aim3/r*pursuit_circle_5.0.log; do
  echo "== $(basename "$f")"
  grep -aE 'velocity mode|BEARING_HOLD|resume|→ AIM|aim timeout|FIRE|bearing hold' "$f" | head -6
  grep -a 'estimator diag' "$f" | tail -1 | grep -o 'obs=[0-9]* inv=[0-9]* acc=[0-9]* reset=[0-9]* init=[0-9]'
done
