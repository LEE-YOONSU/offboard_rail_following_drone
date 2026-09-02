#!/usr/bin/env bash
# One-shot: clean up any previous run, launch the headless sim + rail autoflight,
# and attach a Gazebo GUI window so the flight is visible. Ctrl+C stops everything.
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_DIR}"

echo "[show] cleaning up any previous simulation..."
for pat in "gz sim" "bin/px4" "rail_autoflight.py" "run_autoflight_sim.sh" "run_x500.sh"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" != "$$" ] && kill -9 "$p" 2>/dev/null
  done
done
sleep 3

export YOLO_CONFIG_DIR=/tmp/Ultralytics
export RUN_X500_HEADLESS=1
export ESTIMATOR_SETTLE_SECONDS="${ESTIMATOR_SETTLE_SECONDS:-40}"

gui_pid=""
cleanup() {
  [ -n "${gui_pid}" ] && kill "${gui_pid}" 2>/dev/null
  for pat in "gz sim" "bin/px4" "rail_autoflight.py" "run_x500.sh"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  done
}
trap cleanup EXIT INT TERM

# Bring up the GUI once the world clock is live (physics already running on the
# headless server, so this only adds a viewer).
(
  for _ in $(seq 1 90); do
    gz topic -l 2>/dev/null | grep -q '/world/railway_environment/clock' && break
    sleep 1
  done
  echo "[show] attaching Gazebo GUI window..."
  gz sim -g
) &
gui_pid=$!

./run_autoflight_sim.sh \
  --preflight-timeout 120 --preflight-seconds 3 \
  --speed 1.2 --height 2.5 --altitude-kp 1.2 \
  --confidence 0.40 \
  --center-kp 2.2 --max-lateral-speed 1.6 \
  --heading-kp 2.4 --max-yaw-rate 1.0 \
  --detection-timeout 3.0 --line-loss-abort 25 \
  "$@"
