#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
PX4_MODEL_DIR="${PX4_DIR}/Tools/simulation/gz/models"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
if [[ -d "${PX4_MODEL_DIR}" ]]; then
  export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${PX4_MODEL_DIR}"
fi

exec gz sim -v 3 -s -r "${PROJECT_DIR}/worlds/railway_environment.sdf" "$@"
