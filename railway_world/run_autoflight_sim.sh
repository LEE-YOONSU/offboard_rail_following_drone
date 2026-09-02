#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${PROJECT_DIR}/output/rail_autoflight"
SIM_LOG="${OUTPUT_DIR}/simulation.log"
MODE="execute"

if [[ "${1:-}" == "--dry-run" ]]; then
  MODE="dry-run"
  shift
fi

mkdir -p "${OUTPUT_DIR}"
simulation_pid=""

cleanup() {
  if [[ -n "${simulation_pid}" ]] && kill -0 "${simulation_pid}" 2>/dev/null; then
    kill -INT "${simulation_pid}" 2>/dev/null || true
    wait "${simulation_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[1/3] Gazebo + PX4 SITL starting"
"${PROJECT_DIR}/run_x500.sh" >"${SIM_LOG}" 2>&1 &
simulation_pid=$!

camera_ready=false
for _ in {1..120}; do
  if ! kill -0 "${simulation_pid}" 2>/dev/null; then
    echo "Gazebo/PX4 exited during startup. See ${SIM_LOG}" >&2
    tail -n 30 "${SIM_LOG}" >&2 || true
    exit 1
  fi
  if gz topic -l 2>/dev/null | grep -q '^/x500/rail_down_camera/image$'; then
    camera_ready=true
    break
  fi
  sleep 0.25
done

if [[ "${camera_ready}" != true ]]; then
  echo "Timed out waiting for the X500 camera. See ${SIM_LOG}" >&2
  exit 1
fi

echo "[2/3] Camera ready; waiting for PX4 estimator"
sleep "${ESTIMATOR_SETTLE_SECONDS:-5}"

echo "[3/3] Rail autoflight starting (${MODE})"
echo "Simulation log: ${SIM_LOG}"
if [[ "${MODE}" == "dry-run" ]]; then
  "${PROJECT_DIR}/run_autoflight.sh" "$@"
else
  "${PROJECT_DIR}/run_autoflight.sh" --execute "$@"
fi
