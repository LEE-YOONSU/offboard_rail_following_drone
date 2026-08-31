#!/usr/bin/env bash
set -euo pipefail

status=0

check_command() {
  local command_name="$1"
  local description="$2"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf '[OK] %s: %s\n' "${description}" "$(command -v "${command_name}")"
  else
    printf '[MISSING] %s (%s)\n' "${description}" "${command_name}" >&2
    status=1
  fi
}

check_command gz "Gazebo command"
check_command python3 "Python 3"

if command -v gz >/dev/null 2>&1; then
  gz_sim_major="$(gz sim --versions 2>/dev/null | head -n 1 | cut -d. -f1 || true)"
  printf '[INFO] Gazebo libraries:\n'
  gz sim --versions 2>/dev/null | sed 's/^/  /' || true
  if [[ "${gz_sim_major}" == "8" ]]; then
    printf '[OK] Gazebo Sim 8 (Harmonic)\n'
  else
    printf '[MISMATCH] Expected Gazebo Sim 8 (Harmonic), found major version: %s\n' "${gz_sim_major:-unknown}" >&2
    status=1
  fi
fi

if command -v python3 >/dev/null 2>&1; then
  if python3 - <<'PY'
from PIL import Image  # noqa: F401
from pymavlink import mavutil  # noqa: F401
from gz.msgs10.image_pb2 import Image as GzImage  # noqa: F401
from gz.transport13 import Node  # noqa: F401
PY
  then
    printf '[OK] Python capture dependencies\n'
  else
    printf '[MISSING] One or more Python capture dependencies\n' >&2
    printf '          Install pip packages with: python3 -m pip install -r requirements.txt\n' >&2
    printf '          Gazebo Python bindings come from the Gazebo Harmonic installation.\n' >&2
    status=1
  fi
fi

px4_dir="${PX4_DIR:-${HOME}/PX4-Autopilot}"
if [[ -x "${px4_dir}/build/px4_sitl_default/bin/px4" ]]; then
  printf '[OK] PX4 SITL: %s\n' "${px4_dir}"
else
  printf '[OPTIONAL] PX4 SITL not built at %s\n' "${px4_dir}"
  printf '           Set PX4_DIR when PX4 is installed elsewhere.\n'
fi

exit "${status}"
