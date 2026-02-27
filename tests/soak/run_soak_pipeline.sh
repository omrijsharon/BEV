#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LAUNCHER="${ROOT_DIR}/tools/run_mp_pipeline.sh"
CTL="${ROOT_DIR}/build/bev_ctl"
CFG="${1:-${ROOT_DIR}/config/config.yaml}"
CALIB="${2:-${ROOT_DIR}/config/camera_calibration.yaml}"
DURATION_SEC="${3:-1800}"
OUT="${4:-${ROOT_DIR}/docs/soak_report_$(date +%Y-%m-%d_%H-%M-%S).log}"

if [[ ! -x "${LAUNCHER}" || ! -x "${CTL}" ]]; then
  echo "missing launcher or control binary"
  echo "build first: cmake -S . -B build && cmake --build build -j"
  exit 1
fi

echo "starting soak test for ${DURATION_SEC}s" | tee "${OUT}"
echo "config=${CFG}" | tee -a "${OUT}"

"${LAUNCHER}" "${CFG}" "${CALIB}" >/tmp/bev_soak_launcher.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  "${CTL}" shutdown "${CFG}" >/dev/null 2>&1 || true
  kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 150); do
  if HEALTH_BOOT="$("${CTL}" health "${CFG}" 2>/dev/null)"; then
    if ! grep -q "fallback-shm" <<<"${HEALTH_BOOT}"; then
      break
    fi
  fi
  sleep 0.1
done

START_TS=$(date +%s)
END_TS=$((START_TS + DURATION_SEC))

while [[ $(date +%s) -lt ${END_TS} ]]; do
  NOW=$(date +%s)
  ELAPSED=$((NOW - START_TS))
  echo "---- t=${ELAPSED}s ----" | tee -a "${OUT}"
  if HEALTH_OUT="$("${CTL}" health "${CFG}" 2>/dev/null)"; then
    echo "${HEALTH_OUT}" | tee -a "${OUT}"
  else
    echo "health_unavailable" | tee -a "${OUT}"
  fi
  for role in camera fc bev track map web; do
    PID_FILE="/tmp/bev_${role}.pid"
    if [[ -f "${PID_FILE}" ]]; then
      PID="$(cat "${PID_FILE}" 2>/dev/null || true)"
      if [[ -n "${PID}" && -r "/proc/${PID}/status" ]]; then
        RSS="$(grep -E '^VmRSS:' "/proc/${PID}/status" | awk '{print $2" "$3}')"
        echo "rss_${role}=${RSS}" | tee -a "${OUT}"
      fi
    fi
  done
  sleep 5
done

echo "soak test finished successfully" | tee -a "${OUT}"
