#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CTL="${ROOT_DIR}/build/bev_ctl"
SOAK="${ROOT_DIR}/tests/soak/run_soak_pipeline.sh"
CFG="${1:-${ROOT_DIR}/config/config.yaml}"
CALIB="${2:-${ROOT_DIR}/config/camera_calibration.yaml}"
DURATION="${3:-15}"
OUT="${4:-${ROOT_DIR}/docs/hotpath_profile_$(date +%Y-%m-%d_%H-%M-%S).md}"
TMP_LOG="/tmp/bev_hotpath_profile.log"

if [[ ! -x "${CTL}" || ! -x "${SOAK}" ]]; then
  echo "missing binaries/scripts; build first"
  exit 1
fi

"${SOAK}" "${CFG}" "${CALIB}" "${DURATION}" "${TMP_LOG}" >/dev/null
H="$("${CTL}" health "${CFG}" 2>/dev/null || true)"

{
  echo "# Hotpath Profile Snapshot"
  echo
  echo "- Config: ${CFG}"
  echo "- Duration: ${DURATION}s"
  echo "- Timestamp: $(date -Is)"
  echo
  echo "## Stage KPI"
  echo '```'
  echo "${H}" | grep -E '^stage_'
  echo '```'
  echo
  echo "## Thermal"
  echo '```'
  echo "${H}" | grep -E '^thermal '
  echo '```'
  echo
  echo "## Role Health"
  echo '```'
  echo "${H}" | grep -E '^(camera|fc|bev|track|map|web) '
  echo '```'
} > "${OUT}"

echo "wrote ${OUT}"
