#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  fingerprint_px4_firmware.sh PX4_SOURCE [OUTPUT_DIR] [FIRMWARE_PX4]

Collects a read-only PX4 firmware baseline for the Jetson V0 ledger. It does
not build, flash, fetch, checkout, or modify the supplied source tree.
EOF
}

if [[ $# -lt 1 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

source_arg=$1
output_arg=${2:-"$(pwd)/px4_fingerprint_$(date -u +%Y%m%dT%H%M%SZ)"}
firmware_arg=${3:-}

if [[ ! -d "$source_arg" ]]; then
  echo "ERROR: PX4 source directory does not exist: $source_arg" >&2
  exit 2
fi

source_dir=$(cd "$source_arg" && pwd -P)
if [[ ! -d "$source_dir/.git" && ! -f "$source_dir/.git" ]]; then
  echo "ERROR: not a Git PX4 source tree: $source_dir" >&2
  exit 2
fi
if [[ ! -d "$source_dir/msg" ]]; then
  echo "ERROR: PX4 msg directory is missing: $source_dir/msg" >&2
  exit 2
fi

mkdir -p "$output_arg"
output_dir=$(cd "$output_arg" && pwd -P)
report="$output_dir/px4_firmware_fingerprint.txt"
manifest="$output_dir/relevant_files_sha256.txt"
patch_file="$output_dir/v1.15.4_relevant_changes.patch"

git_px4() {
  git -C "$source_dir" "$@"
}

hash_tree() {
  local tree=$1
  if [[ ! -d "$tree" ]]; then
    printf 'MISSING'
    return
  fi
  find "$tree" -type f -print0 \
    | LC_ALL=C sort -z \
    | xargs -0 -r sha256sum \
    | sha256sum \
    | awk '{print $1}'
}

board_file="$source_dir/boards/px4/fmu-v6x/default.px4board"
dds_file="$source_dir/src/modules/uxrce_dds_client/dds_topics.yaml"

{
  echo "PX4 firmware fingerprint"
  echo "collected_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "source_dir=$source_dir"
  echo "expected_hardware=Holybro Pixhawk 6X Mini"
  echo "expected_board_family=FMUv6X"
  echo "expected_build_target=px4_fmu-v6x_default"
  echo "declared_base_release=PX4 v1.15.4 (custom modifications reported)"
  echo "git_head=$(git_px4 rev-parse HEAD)"
  echo "git_describe=$(git_px4 describe --always --tags --dirty 2>/dev/null || true)"
  echo "git_branch=$(git_px4 symbolic-ref --short -q HEAD 2>/dev/null || echo DETACHED)"
  echo "v1.15.4_ref=$(git_px4 rev-parse v1.15.4^{commit} 2>/dev/null || echo UNAVAILABLE_LOCALLY)"
  echo "msg_tree_sha256=$(hash_tree "$source_dir/msg")"
  if [[ -f "$board_file" ]]; then
    echo "fmu_v6x_board_sha256=$(sha256sum "$board_file" | awk '{print $1}')"
  else
    echo "fmu_v6x_board_sha256=MISSING"
  fi
  if [[ -f "$dds_file" ]]; then
    echo "dds_topics_sha256=$(sha256sum "$dds_file" | awk '{print $1}')"
  else
    echo "dds_topics_sha256=MISSING"
  fi
  if [[ -n "$firmware_arg" ]]; then
    if [[ ! -f "$firmware_arg" ]]; then
      echo "firmware_sha256=ERROR_NOT_FOUND:$firmware_arg"
    else
      firmware_path=$(cd "$(dirname "$firmware_arg")" && pwd -P)/$(basename "$firmware_arg")
      echo "firmware_path=$firmware_path"
      echo "firmware_sha256=$(sha256sum "$firmware_path" | awk '{print $1}')"
    fi
  else
    echo "firmware_sha256=NOT_SUPPLIED"
  fi
  echo
  echo "[git status --short]"
  git_px4 status --short --untracked-files=all
  echo
  echo "[submodules]"
  git_px4 submodule status --recursive 2>/dev/null || true
  echo
  echo "[recent commits]"
  git_px4 log -n 20 --date=iso-strict --pretty='format:%H%x09%ad%x09%an%x09%s'
} > "$report"

{
  find "$source_dir/msg" -type f -print0
  [[ -f "$board_file" ]] && printf '%s\0' "$board_file"
  [[ -f "$dds_file" ]] && printf '%s\0' "$dds_file"
} | LC_ALL=C sort -z | xargs -0 -r sha256sum > "$manifest"

if git_px4 rev-parse --verify v1.15.4^{commit} >/dev/null 2>&1; then
  git_px4 diff --submodule=short --binary v1.15.4 -- \
    msg \
    src/modules/uxrce_dds_client \
    boards/px4/fmu-v6x \
    ROMFS/px4fmu_common/init.d \
    src/modules/commander \
    src/modules/navigator \
    src/modules/mc_pos_control \
    > "$patch_file"
else
  printf '%s\n' 'v1.15.4 ref is not present locally; no baseline patch generated.' > "$patch_file"
fi

echo "Fingerprint written to: $output_dir"
echo "Review the report, manifest, and patch before selecting px4_msgs or enabling live control."
