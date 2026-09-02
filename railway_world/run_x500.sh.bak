#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_BUILD="${PX4_DIR}/build/px4_sitl_default"
WORLD_FILE="${PROJECT_DIR}/worlds/railway_environment.sdf"

if [[ ! -x "${PX4_BUILD}/bin/px4" ]]; then
  echo "PX4 SITL binary not found: ${PX4_BUILD}/bin/px4" >&2
  echo "Build it first with: cd ${PX4_DIR} && make px4_sitl gz_x500" >&2
  exit 1
fi

# Match Gazebo worker process names exactly. A full-command regex can match the
# pgrep command itself on some procps versions and falsely report Gazebo running.
if pgrep -x 'gz sim server' >/dev/null 2>&1 || \
   pgrep -x 'gz sim gui' >/dev/null 2>&1 || \
   pgrep -x 'gzserver' >/dev/null 2>&1 || \
   pgrep -x 'gzclient' >/dev/null 2>&1; then
  echo "Another Gazebo simulation is already running." >&2
  echo "Close it first, then run ./run_x500.sh so the camera Sensors system is enabled." >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PX4_DIR}/Tools/simulation/gz/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

gazebo_pid=""
cleanup() {
  if [[ -n "${gazebo_pid}" ]] && kill -0 "${gazebo_pid}" 2>/dev/null; then
    kill "${gazebo_pid}" 2>/dev/null || true
    wait "${gazebo_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=xcb
fi

# The normal run.sh stays camera-free. This launch retains the Sensors system only
# for the single camera mounted on the X500.
gz sim -v 3 -r "${WORLD_FILE}" &
gazebo_pid=$!

world_ready=false
for _ in {1..150}; do
  if gz topic -l 2>/dev/null | grep -q '^/world/railway_environment/clock$'; then
    world_ready=true
    break
  fi
  if ! kill -0 "${gazebo_pid}" 2>/dev/null; then
    echo "Gazebo exited before the railway world became ready." >&2
    exit 1
  fi
  sleep 0.1
done

if [[ "${world_ready}" != true ]]; then
  echo "Timed out waiting for the Gazebo railway world." >&2
  exit 1
fi

echo "PX4 attaching to existing Entity Tree model: x500"
echo "Down camera topic: /x500/rail_down_camera/image"
echo "Press Ctrl+C in this terminal to stop PX4 and Gazebo together."

cd "${PX4_BUILD}/rootfs"
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_NAME=x500 \
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=railway_environment \
"${PX4_BUILD}/bin/px4"
