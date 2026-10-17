#!/bin/bash
set -eu
if [ "$(id -u)" != 0 ]; then echo 'Run with sudo; password is entered only in your SSH terminal.' >&2; exit 1; fi
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
install -d -o root -g root -m 755 /usr/local/libexec
install -o root -g root -m 755 "$src/clock_sync_helper.py" /usr/local/libexec/zed-clock-sync
tmp=$(mktemp)
trap 'rm -f -- "$tmp"' EXIT
printf '%s\n' 'uavlab ALL=(root) NOPASSWD: /usr/local/libexec/zed-clock-sync ""' > "$tmp"
/usr/sbin/visudo -cf "$tmp"
install -o root -g root -m 440 "$tmp" /etc/sudoers.d/zed-clock-sync
echo 'Installed fixed UTC/RTC helper only. Reconnect the panel while camera is stopped to synchronize from PC.'
