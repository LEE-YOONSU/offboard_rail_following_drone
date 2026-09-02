#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON}" ]]; then
  echo "Virtual environment not found: ${PYTHON}" >&2
  echo "Create it and install requirements plus ultralytics first." >&2
  exit 1
fi

exec "${PYTHON}" "${PROJECT_DIR}/tools/rail_autoflight.py" "$@"
