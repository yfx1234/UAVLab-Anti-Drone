#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/home/uavlab/Jetson_ROS2_V0}"
GUARD_SOURCE="${PROJECT_ROOT}/deploy/falcon_clock_guard.sh"
UNIT_DIR="${PROJECT_ROOT}/deploy/systemd"
MIN_VALID_EPOCH=1704067200

for path in \
    "$GUARD_SOURCE" \
    "$UNIT_DIR/falcon-clock-guard.service" \
    "$UNIT_DIR/falcon-clock-save.service" \
    "$UNIT_DIR/falcon-clock-save.timer"; do
    [[ -f "$path" ]] || { echo "missing required file: $path" >&2; exit 1; }
done

current_epoch="$(date +%s)"
seed_epoch="${SEED_EPOCH:-}"
if (( current_epoch < MIN_VALID_EPOCH )) && [[ -z "$seed_epoch" ]]; then
    # scp -p preserves the deployment file's workstation timestamp. This is a
    # trustworthy lower bound for first install when both onboard RTCs are 1970.
    seed_epoch="$(stat -c %Y "$GUARD_SOURCE")"
    echo "Using preserved deployment timestamp as initial lower bound: ${seed_epoch}"
fi

if [[ -n "$seed_epoch" ]]; then
    [[ "$seed_epoch" =~ ^[0-9]+$ ]] && (( seed_epoch >= MIN_VALID_EPOCH )) \
        || { echo "SEED_EPOCH/deployment timestamp must be at or after 2024-01-01" >&2; exit 2; }
    if (( current_epoch < seed_epoch )); then
        sudo date -u --set="@${seed_epoch}"
    fi
fi

sudo install -o root -g root -m 0755 "$GUARD_SOURCE" /usr/local/sbin/falcon-clock-guard
sudo install -o root -g root -m 0644 "$UNIT_DIR/falcon-clock-guard.service" /etc/systemd/system/falcon-clock-guard.service
sudo install -o root -g root -m 0644 "$UNIT_DIR/falcon-clock-save.service" /etc/systemd/system/falcon-clock-save.service
sudo install -o root -g root -m 0644 "$UNIT_DIR/falcon-clock-save.timer" /etc/systemd/system/falcon-clock-save.timer

sudo mkdir -p /etc/systemd/timesyncd.conf.d
printf '%s\n' \
    '[Time]' \
    'NTP=ntp.aliyun.com ntp.tencent.com cn.pool.ntp.org' \
    'FallbackNTP=0.pool.ntp.org 1.pool.ntp.org' \
    | sudo tee /etc/systemd/timesyncd.conf.d/50-falcon.conf >/dev/null

sudo systemctl daemon-reload
sudo systemctl unmask systemd-timesyncd.service
sudo systemctl enable systemd-timesyncd.service
sudo systemctl reenable falcon-clock-guard.service falcon-clock-save.timer
sudo systemctl start falcon-clock-guard.service
sudo systemctl restart systemd-timesyncd.service
sudo systemctl start falcon-clock-save.timer

echo
sudo /usr/local/sbin/falcon-clock-guard status
echo
systemctl --no-pager --full status falcon-clock-guard.service falcon-clock-save.timer systemd-timesyncd.service || true
