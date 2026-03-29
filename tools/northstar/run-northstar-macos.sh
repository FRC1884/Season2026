#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 /path/to/RobotCode2026Public/northstar"
  exit 1
fi

NORTHSTAR_DIR="$1"

if [[ ! -d "$NORTHSTAR_DIR" ]]; then
  echo "northstar directory not found: $NORTHSTAR_DIR"
  exit 1
fi

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$NORTHSTAR_DIR"

if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi

source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

python __init__.py \
  --config "$CONFIG_DIR/config-left.json" \
  --calibration "$CONFIG_DIR/calibration-left.yml" &

python __init__.py \
  --config "$CONFIG_DIR/config-right.json" \
  --calibration "$CONFIG_DIR/calibration-right.yml" &

python __init__.py \
  --config "$CONFIG_DIR/config-side.json" \
  --calibration "$CONFIG_DIR/calibration-side.yml" &

wait
