#!/usr/bin/env bash
# 项目脚本迁移入库：~/xx.{sh,py} → interceptor_ws/tools/sim/，路径改指
# 新工作区；一次性迁移脚本（migrate_out/clone_to_wsl/list_scripts）除外
set -e
DST=~/interceptor_ws/tools/sim
mkdir -p "$DST"
SKIP="migrate_out.sh clone_to_wsl.sh list_scripts.sh"
for f in ~/*.sh ~/*.py; do
  [ -f "$f" ] || continue
  b=$(basename "$f")
  case " $SKIP " in *" $b "*) continue;; esac
  sed 's|interceptor_ws|interceptor_ws|g' "$f" > "$DST/$b"
done
# 统一可执行位
chmod +x "$DST"/*.sh
ls "$DST" | wc -l
grep -rl 'interceptor_ws' "$DST" | wc -l
