#!/usr/bin/env bash

set -Eeuo pipefail

PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
PX4_BIN="${BUILD_DIR}/bin/px4"
PX4_ROOTFS="${BUILD_DIR}/etc"
WORLD_FILE="Tools/simulation/gz/worlds/default.sdf"
WORLD_NAME="default"
LOG_ROOT="${DUAL_SIM_LOG_DIR:-${HOME}/px4sim/logs/dual}"
LOG_DIR="${LOG_ROOT}/$(date +%Y%m%d_%H%M%S)"
ULOG_OUTPUT_DIR="${DUAL_SIM_ULOG_OUTPUT_DIR:-${LOG_DIR}}"

TARGET_POSE="${TARGET_POSE:-0,0,0.01,0,0,1.570796}"
PURSUER_POSE="${PURSUER_POSE:--5,0,0.01,0,0,1.570796}"
TARGET_AUTOSTART="${TARGET_AUTOSTART:-4001}"
TARGET_SIM_MODEL="${TARGET_SIM_MODEL:-gz_x500}"
PURSUER_AUTOSTART="${PURSUER_AUTOSTART:-22001}"
PURSUER_SIM_MODEL="${PURSUER_SIM_MODEL:-gz_my_quad_real}"
# 僚机（PX4 实例 2/3）：同样使用 22001 机型（16 m/s 包络）；默认在实机后两翼
WINGMAN_L_POSE="${WINGMAN_L_POSE:-30,-40,0.01,0,0,1.570796}"
WINGMAN_R_POSE="${WINGMAN_R_POSE:--30,-40,0.01,0,0,1.570796}"
WINGMAN_AUTOSTART="${WINGMAN_AUTOSTART:-22001}"
WINGMAN_SIM_MODEL="${WINGMAN_SIM_MODEL:-gz_my_quad_real}"
HEADLESS="${HEADLESS:-false}"

GAZEBO_LAUNCHER_PID=""
GAZEBO_SERVER_PIDS=()
PX4_PIDS=()

log() { printf '[startpx4-quad] %s\n' "$*"; }

stop_pids() {
    local label=$1
    shift
    local pids=("$@")
    ((${#pids[@]} == 0)) && return
    log "Stopping ${label}: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true
    for _ in {1..40}; do
        local alive=false
        for pid in "${pids[@]}"; do
            kill -0 "${pid}" 2>/dev/null && alive=true && break
        done
        [[ "${alive}" == false ]] && return
        sleep 0.25
    done
    log "WARNING: ${label} did not stop within 10 seconds; no SIGKILL was used."
}

archive_ulog() {
    local instance=$1 role=$2
    local instance_log="${LOG_DIR}/px4_${instance}_${role}.log"
    local relative_path
    relative_path="$(
        sed -n 's/.*Opened full log file: //p' "${instance_log}" 2>/dev/null |
            tail -n 1 |
            tr -d '\r'
    )"
    if [[ -z "${relative_path}" ]]; then
        log "WARNING: PX4 ${instance} ULog path was not reported."
        return
    fi
    local source_path="${BUILD_DIR}/instance_${instance}/${relative_path#./}"
    if [[ ! -s "${source_path}" ]]; then
        log "WARNING: PX4 ${instance} ULog is missing: ${source_path}"
        return
    fi
    mkdir -p "${ULOG_OUTPUT_DIR}"
    cp -p "${source_path}" "${ULOG_OUTPUT_DIR}/px4_${instance}.ulg"
    log "Archived PX4 ${instance} ULog to ${ULOG_OUTPUT_DIR}/px4_${instance}.ulg"
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    stop_pids "PX4 instance(s)" "${PX4_PIDS[@]}"
    archive_ulog 0 target || true
    archive_ulog 1 pursuer || true
    archive_ulog 2 wingman_l || true
    archive_ulog 3 wingman_r || true
    stop_pids "owned Gazebo server(s)" "${GAZEBO_SERVER_PIDS[@]}"
    if [[ -n "${GAZEBO_LAUNCHER_PID}" ]] && kill -0 "${GAZEBO_LAUNCHER_PID}" 2>/dev/null; then
        stop_pids "Gazebo launcher" "${GAZEBO_LAUNCHER_PID}"
    fi
    log "Logs retained in ${LOG_DIR}"
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

for required in "${PX4_DIR}" "${BUILD_DIR}" "${PX4_ROOTFS}"; do
    [[ -d "${required}" ]] || { log "ERROR: Missing directory: ${required}"; exit 1; }
done
[[ -x "${PX4_BIN}" ]] || { log "ERROR: Run 'make px4_sitl_default' first."; exit 1; }
[[ -f "${PX4_DIR}/${WORLD_FILE}" ]] || { log "ERROR: Missing ${WORLD_FILE}"; exit 1; }

if pgrep -x px4 >/dev/null 2>&1; then
    log "ERROR: PX4 is already running; stop it before starting a new quad simulation."
    pgrep -af '(^|/)px4( |$)' || true
    exit 2
fi

mapfile -t old_gz_pids < <(pgrep -f 'gz sim server' || true)
if ((${#old_gz_pids[@]})); then
    stop_pids "existing Gazebo server(s)" "${old_gz_pids[@]}"
    pgrep -f 'gz sim server' >/dev/null 2>&1 && {
        log "ERROR: Existing Gazebo server remains; no new simulation was started."
        exit 3
    }
fi

ss -lun 2>/dev/null | grep -qE '(^|[[:space:]])[^[:space:]]*:8888[[:space:]]' || {
    log "ERROR: Start 'MicroXRCEAgent udp4 -p 8888' first."
    exit 4
}

mkdir -p "${LOG_DIR}" "${BUILD_DIR}/instance_0" "${BUILD_DIR}/instance_1" \
  "${BUILD_DIR}/instance_2" "${BUILD_DIR}/instance_3"
cd "${PX4_DIR}"

log "Starting Gazebo world '${WORLD_NAME}' (headless=${HEADLESS})..."
if [[ "${HEADLESS}" == true ]]; then
    gz sim -r -s "${WORLD_FILE}" >"${LOG_DIR}/gazebo.log" 2>&1 &
else
    gz sim -r "${WORLD_FILE}" >"${LOG_DIR}/gazebo.log" 2>&1 &
fi
GAZEBO_LAUNCHER_PID=$!

gazebo_ready=false
for second in {1..30}; do
    mapfile -t GAZEBO_SERVER_PIDS < <(pgrep -f 'gz sim server' || true)
    if gz service -l 2>/dev/null | grep -qx "/world/${WORLD_NAME}/control"; then
        gazebo_ready=true
        log "Gazebo is ready after ${second} second(s)."
        break
    fi
    sleep 1
done
[[ "${gazebo_ready}" == true ]] || { log "ERROR: Gazebo startup timed out."; exit 5; }

start_px4() {
    local instance=$1 pose=$2 role=$3 autostart=$4 model=$5
    local work_dir="${BUILD_DIR}/instance_${instance}"
    local instance_log="${LOG_DIR}/px4_${instance}_${role}.log"
    log "Starting PX4 ${instance} (${role}, ${model}/${autostart}) at ${pose}..."
    (
        cd "${work_dir}"
        PX4_SYS_AUTOSTART="${autostart}" \
        PX4_SIM_MODEL="${model}" \
        PX4_GZ_MODEL_POSE="${pose}" \
        PX4_GZ_STANDALONE=1 \
        "${PX4_BIN}" -i "${instance}" -d "${PX4_ROOTFS}"
    ) >"${instance_log}" 2>&1 &
    PX4_PIDS+=("$!")
}

start_px4 0 "${TARGET_POSE}" target "${TARGET_AUTOSTART}" "${TARGET_SIM_MODEL}"
sleep 2
start_px4 1 "${PURSUER_POSE}" pursuer "${PURSUER_AUTOSTART}" "${PURSUER_SIM_MODEL}"
sleep 2
start_px4 2 "${WINGMAN_L_POSE}" wingman_l "${WINGMAN_AUTOSTART}" "${WINGMAN_SIM_MODEL}"
sleep 2
start_px4 3 "${WINGMAN_R_POSE}" wingman_r "${WINGMAN_AUTOSTART}" "${WINGMAN_SIM_MODEL}"

all_ready=false
for _ in {1..40}; do
    if grep -q 'Startup script returned successfully' "${LOG_DIR}/px4_0_target.log" && \
       grep -q 'Startup script returned successfully' "${LOG_DIR}/px4_1_pursuer.log" && \
       grep -q 'Startup script returned successfully' "${LOG_DIR}/px4_2_wingman_l.log" && \
       grep -q 'Startup script returned successfully' "${LOG_DIR}/px4_3_wingman_r.log"; then
        all_ready=true
        break
    fi
    for pid in "${PX4_PIDS[@]}"; do kill -0 "${pid}" 2>/dev/null || exit 6; done
    sleep 1
done
[[ "${all_ready}" == true ]] || { log "ERROR: PX4 startup timed out."; exit 7; }

log "All four PX4 instances are ready."
log "Target: /fmu, ${TARGET_SIM_MODEL}/${TARGET_AUTOSTART}, ${TARGET_POSE}"
log "Pursuer: /px4_1/fmu, ${PURSUER_SIM_MODEL}/${PURSUER_AUTOSTART}, ${PURSUER_POSE}"
log "Wingman L: /px4_2/fmu, ${WINGMAN_SIM_MODEL}/${WINGMAN_AUTOSTART}, ${WINGMAN_L_POSE}"
log "Wingman R: /px4_3/fmu, ${WINGMAN_SIM_MODEL}/${WINGMAN_AUTOSTART}, ${WINGMAN_R_POSE}"
log "Press Ctrl+C to stop all four PX4 instances and Gazebo."

while true; do
    for pid in "${PX4_PIDS[@]}"; do
        kill -0 "${pid}" 2>/dev/null || { log "ERROR: PX4 ${pid} exited."; exit 8; }
    done
    sleep 1
done
