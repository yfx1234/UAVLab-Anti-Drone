#!/usr/bin/env bash
# headon 轮 AIM 进入情况检查
for f in ~/netgun_logs/matrix_aim2/r*headon*.log; do
  n=$(grep -ac '→ AIM' "$f")
  seq=$(grep -aE 'velocity mode|BEARING_HOLD|→ AIM|FIRE|est lost' "$f" | head -5 | tr '\n' ' | ')
  echo "$(basename "$f") AIM_entries=$n"
  echo "  $seq"
done
