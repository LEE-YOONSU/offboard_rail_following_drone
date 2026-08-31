#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_MODEL_DIR="${PX4_DIR}/Tools/simulation/gz/models"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
if [[ -d "${PX4_MODEL_DIR}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${PX4_MODEL_DIR}"
fi

WORLD_FILE="${PROJECT_DIR}/worlds/railway_environment.sdf"

# Server-only mode keeps the multimodal synthetic sensors enabled.
for arg in "$@"; do
  if [[ "$arg" == "-s" || "$arg" == "--server-only" ]]; then
    exec gz sim -v 3 "$WORLD_FILE" "$@"
  fi
done

# Interactive viewing removes only the unused sensor system; the optimized detailed
# track now loads directly without hundreds of nested sleeper model entities.
GUI_WORLD="$(mktemp /tmp/railway_environment_gui.XXXXXX.sdf)"
trap 'rm -f "$GUI_WORLD"' EXIT INT TERM
sed \
  -e '/gz-sim-sensors-system/d' \
  "$WORLD_FILE" > "$GUI_WORLD"

if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
  export QT_QPA_PLATFORM=xcb
fi

gz sim -v 3 -r "$GUI_WORLD" "$@"
