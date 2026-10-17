#!/usr/bin/env bash
set -euo pipefail

# Keep the Jetson wall clock usable when the carrier RTC has no backup power.
# This is deliberately a lower-bound clock: it never moves a valid clock
# backwards and restores the newest trustworthy timestamp seen on disk.

STATE_DIR="${FALCON_CLOCK_STATE_DIR:-/var/lib/falcon-clock}"
STATE_FILE="${STATE_DIR}/last_good_epoch"
TIMESYNCD_CLOCK="/var/lib/systemd/timesync/clock"
MIN_VALID_EPOCH="${FALCON_CLOCK_MIN_VALID_EPOCH:-1704067200}" # 2024-01-01 UTC

log()
{
    printf '[falcon-clock] %s\n' "$*"
}

valid_epoch()
{
    [[ "${1:-}" =~ ^[0-9]+$ ]] && (( $1 >= MIN_VALID_EPOCH ))
}

state_epoch()
{
    local value=""
    if [[ -r "$STATE_FILE" ]]; then
        read -r value < "$STATE_FILE" || true
    fi
    valid_epoch "$value" && printf '%s\n' "$value"
}

timesync_epoch()
{
    local value=""
    if [[ -e "$TIMESYNCD_CLOCK" ]]; then
        value="$(stat -c %Y "$TIMESYNCD_CLOCK" 2>/dev/null || true)"
    fi
    valid_epoch "$value" && printf '%s\n' "$value"
}

save_clock()
{
    local now tmp
    now="$(date +%s)"
    if ! valid_epoch "$now"; then
        log "not saving invalid system epoch ${now}"
        return 2
    fi

    install -d -o root -g root -m 0755 "$STATE_DIR"
    tmp="${STATE_FILE}.tmp"
    printf '%s\n' "$now" > "$tmp"
    chmod 0644 "$tmp"
    mv -f "$tmp" "$STATE_FILE"
    log "saved epoch ${now} ($(date --iso-8601=seconds))"
}

restore_clock()
{
    local now candidate value source="none"
    now="$(date +%s)"
    candidate=0

    value="$(state_epoch || true)"
    if valid_epoch "$value" && (( value > candidate )); then
        candidate="$value"
        source="$STATE_FILE"
    fi

    value="$(timesync_epoch || true)"
    if valid_epoch "$value" && (( value > candidate )); then
        candidate="$value"
        source="$TIMESYNCD_CLOCK"
    fi

    if valid_epoch "$now"; then
        log "system clock already valid: $(date --iso-8601=seconds)"
        save_clock
        return 0
    fi

    if valid_epoch "$candidate"; then
        date -u --set="@${candidate}"
        log "restored lower-bound clock from ${source}: $(date --iso-8601=seconds)"
        save_clock
        return 0
    fi

    log "WARNING: no trustworthy persisted time; wall clock remains invalid"
    return 0
}

show_status()
{
    local now saved synced="no"
    now="$(date +%s)"
    saved="$(state_epoch || true)"
    if command -v timedatectl >/dev/null 2>&1; then
        synced="$(timedatectl show -p NTPSynchronized --value 2>/dev/null || printf 'unknown')"
    fi
    printf 'wall_epoch=%s\n' "$now"
    printf 'wall_iso=%s\n' "$(date --iso-8601=seconds)"
    printf 'wall_valid=%s\n' "$(valid_epoch "$now" && printf true || printf false)"
    printf 'saved_epoch=%s\n' "${saved:-none}"
    printf 'ntp_synchronized=%s\n' "$synced"
}

case "${1:-status}" in
    restore) restore_clock ;;
    save) save_clock ;;
    status) show_status ;;
    *)
        echo "usage: $0 {restore|save|status}" >&2
        exit 64
        ;;
esac
