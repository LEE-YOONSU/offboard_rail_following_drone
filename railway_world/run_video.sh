#!/usr/bin/env bash
# Headless flight + record two MP4s: 3rd-person overview and drone-eye (YOLO overlay).
# Nothing here needs a GUI or GPU rendering to a window - only headless sensor
# rendering, which works on this WSL setup. Ctrl+C stops and still assembles.
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_DIR}"
OUT="${PROJECT_DIR}/output/rail_autoflight"
PY="${PROJECT_DIR}/.venv/bin/python"

echo "[video] cleaning up previous run..."
for pat in "gz sim" "bin/px4" "rail_autoflight.py" "grab_overview.py" "run_autoflight_sim.sh" "run_x500.sh"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done
done
sleep 3
rm -rf "${OUT}/overlay_frames" "${OUT}/overview_frames"

export YOLO_CONFIG_DIR=/tmp/Ultralytics
export RUN_X500_HEADLESS=1
export ESTIMATOR_SETTLE_SECONDS="${ESTIMATOR_SETTLE_SECONDS:-40}"

grab_pid=""
sim_pid=""
cleanup() {
  [ -n "${grab_pid}" ] && kill -INT "${grab_pid}" 2>/dev/null
  sleep 1
  for pat in "gz sim" "bin/px4" "rail_autoflight.py" "grab_overview.py" "run_x500.sh"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null; done
  done
}
trap 'cleanup; assemble' EXIT INT TERM

assemble() {
  echo "[video] assembling MP4s..."
  "${PY}" tools/make_video.py "${PROJECT_DIR}/output/overview_frames" "${OUT}/overview.mp4" 15 || true
  "${PY}" tools/make_video.py "${OUT}/overlay_frames"  "${OUT}/drone_eye.mp4" 15 || true
  echo
  echo "[video] done. Open in Windows:"
  echo "  C:\\Users\\SSDNU\\offboard_rail_following_drone\\railway_world\\output\\rail_autoflight\\overview.mp4"
  echo "  C:\\Users\\SSDNU\\offboard_rail_following_drone\\railway_world\\output\\rail_autoflight\\drone_eye.mp4"
}

echo "[1/4] Gazebo (headless) + PX4 SITL"
RUN_X500_HEADLESS=1 ./run_x500.sh >"${OUT}/simulation.log" 2>&1 &
sim_pid=$!

echo "[2/4] waiting for camera topic..."
for _ in $(seq 1 180); do
  kill -0 "${sim_pid}" 2>/dev/null || { echo "sim exited early, see simulation.log"; exit 1; }
  gz topic -l 2>/dev/null | grep -q '^/x500/rail_down_camera/image$' && break
  sleep 0.5
done

echo "[3/4] recording overview + settling estimator (${ESTIMATOR_SETTLE_SECONDS}s)"
"${PY}" tools/grab_overview.py &
grab_pid=$!
sleep "${ESTIMATOR_SETTLE_SECONDS}"

echo "[4/4] rail autoflight (execute)"
./run_autoflight.sh --execute \
  --preflight-timeout 120 --preflight-seconds 3 \
  --speed 1.2 --height 2.5 --altitude-kp 1.2 \
  --confidence 0.40 \
  --center-kp 2.2 --max-lateral-speed 1.6 \
  --heading-kp 2.4 --max-yaw-rate 1.0 \
  --detection-timeout 3.0 --line-loss-abort 25 \
  "$@"
