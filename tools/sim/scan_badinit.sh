#!/usr/bin/env bash
# 坏 boot 扫描：estimator 全程 100% 拒收（inv==obs 且 init=0）的轮
for f in ~/netgun_logs/matrix_aim4/r*.log; do
  last=$(grep -a 'estimator diag' "$f" | tail -1)
  case "$last" in
    *"init=0"*)
      echo "BAD_INIT $(basename "$f") :: $last"
      ;;
  esac
done
echo "--- scan done ---"
