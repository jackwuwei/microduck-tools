#!/usr/bin/env bash
set -euo pipefail
CALIBRATION_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CALIBRATION_PYTHON="${MICRODUCK_PYTHON:-$CALIBRATION_ROOT/.venv/bin/python}"
if [[ ! -x "$CALIBRATION_PYTHON" ]]; then
  echo '请先按 README 创建 .venv 并安装 requirements.txt，或通过 MICRODUCK_PYTHON 指定 Python。' >&2
  exit 1
fi
exec "$CALIBRATION_PYTHON" "$CALIBRATION_ROOT/tools/xl330-calibration/server.py" "$@"
