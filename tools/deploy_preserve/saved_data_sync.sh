#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  saved_data_sync.sh <pull|backup|push|verify> --robot-host HOST --robot-data-root PATH --robot-autos-root PATH --local-data-root PATH --local-autos-root PATH --cache-root PATH
EOF
}

ACTION="${1:-}"
shift || true

ROBOT_HOST=""
ROBOT_DATA_ROOT=""
ROBOT_AUTOS_ROOT=""
LOCAL_DATA_ROOT=""
LOCAL_AUTOS_ROOT=""
CACHE_ROOT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-host)
      ROBOT_HOST="$2"
      shift 2
      ;;
    --robot-data-root)
      ROBOT_DATA_ROOT="$2"
      shift 2
      ;;
    --robot-autos-root)
      ROBOT_AUTOS_ROOT="$2"
      shift 2
      ;;
    --local-data-root)
      LOCAL_DATA_ROOT="$2"
      shift 2
      ;;
    --local-autos-root)
      LOCAL_AUTOS_ROOT="$2"
      shift 2
      ;;
    --cache-root)
      CACHE_ROOT="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "${ACTION}" ]]; then
  usage
  exit 2
fi

if [[ -z "${CACHE_ROOT}" || -z "${LOCAL_DATA_ROOT}" || -z "${LOCAL_AUTOS_ROOT}" ]]; then
  echo "Missing required local paths" >&2
  usage
  exit 2
fi

ROBOT_CACHE_ROOT="${CACHE_ROOT}/robot"
BACKUP_ROOT="${CACHE_ROOT}/backups"

sync_from_remote() {
  local remote_path="$1"
  local local_path="$2"

  mkdir -p "${local_path}"
  if command -v rsync >/dev/null 2>&1; then
    rsync -az --delete --ignore-missing-args "${ROBOT_HOST}:${remote_path}/" "${local_path}/" || true
  else
    rm -rf "${local_path}"
    mkdir -p "${local_path}"
    scp -r "${ROBOT_HOST}:${remote_path}/." "${local_path}/" 2>/dev/null || true
  fi
}

sync_to_remote() {
  local local_path="$1"
  local remote_path="$2"

  ssh "${ROBOT_HOST}" "mkdir -p '${remote_path}'"
  if command -v rsync >/dev/null 2>&1; then
    rsync -az --delete "${local_path}/" "${ROBOT_HOST}:${remote_path}/"
  else
    scp -r "${local_path}/." "${ROBOT_HOST}:${remote_path}/"
  fi
}

case "${ACTION}" in
  pull)
    if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_DATA_ROOT}" || -z "${ROBOT_AUTOS_ROOT}" ]]; then
      echo "Pull requires robot host and remote roots" >&2
      exit 2
    fi
    mkdir -p "${ROBOT_CACHE_ROOT}"
    sync_from_remote "${ROBOT_DATA_ROOT}" "${ROBOT_CACHE_ROOT}/operatorboard-data"
    sync_from_remote "${ROBOT_AUTOS_ROOT}" "${ROBOT_CACHE_ROOT}/pathplana-autos"
    ;;
  backup)
    timestamp="$(date -u +%Y%m%d-%H%M%S)"
    mkdir -p "${BACKUP_ROOT}"
    if [[ -d "${LOCAL_DATA_ROOT}" ]]; then
      cp -R "${LOCAL_DATA_ROOT}" "${BACKUP_ROOT}/local-operatorboard-data-${timestamp}"
    fi
    if [[ -d "${LOCAL_AUTOS_ROOT}" ]]; then
      cp -R "${LOCAL_AUTOS_ROOT}" "${BACKUP_ROOT}/local-pathplana-autos-${timestamp}"
    fi
    ;;
  push)
    if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_DATA_ROOT}" || -z "${ROBOT_AUTOS_ROOT}" ]]; then
      echo "Push requires robot host and remote roots" >&2
      exit 2
    fi
    mkdir -p "${LOCAL_DATA_ROOT}" "${LOCAL_AUTOS_ROOT}"
    sync_to_remote "${LOCAL_DATA_ROOT}" "${ROBOT_DATA_ROOT}"
    sync_to_remote "${LOCAL_AUTOS_ROOT}" "${ROBOT_AUTOS_ROOT}"
    ;;
  verify)
    if [[ -z "${ROBOT_HOST}" || -z "${ROBOT_DATA_ROOT}" || -z "${ROBOT_AUTOS_ROOT}" ]]; then
      echo "Verify requires robot host and remote roots" >&2
      exit 2
    fi
    ssh "${ROBOT_HOST}" "\
      set -euo pipefail; \
      test -d '${ROBOT_DATA_ROOT}'; \
      test -f '${ROBOT_DATA_ROOT}/joystick-mappings.json'; \
      test -f '${ROBOT_DATA_ROOT}/subsystem-descriptions.json'; \
      test -d '${ROBOT_AUTOS_ROOT}'"
    ;;
  *)
    echo "Unknown action: ${ACTION}" >&2
    usage
    exit 2
    ;;
esac
