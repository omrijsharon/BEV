#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LAUNCHER="${ROOT_DIR}/tools/run_mp_pipeline.sh"
CTL="${ROOT_DIR}/build/bev_ctl"
CFG="${1:-${ROOT_DIR}/config/config.yaml}"
CALIB="${2:-${ROOT_DIR}/config/camera_calibration.yaml}"
OUT="${3:-${ROOT_DIR}/docs/parity_report_$(date +%Y-%m-%d_%H-%M-%S).md}"

if [[ ! -x "${LAUNCHER}" || ! -x "${CTL}" ]]; then
  echo "missing launcher or control binary"
  echo "build first: cmake -S . -B build && cmake --build build -j"
  exit 1
fi

mkdir -p "$(dirname "${OUT}")"

"${LAUNCHER}" "${CFG}" "${CALIB}" >/tmp/bev_parity_launcher.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  "${CTL}" shutdown "${CFG}" >/dev/null 2>&1 || true
  kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 150); do
  if H="$("${CTL}" health "${CFG}" 2>/dev/null)"; then
    if ! grep -q "fallback-shm" <<<"${H}"; then
      break
    fi
  fi
  sleep 0.1
done

HEALTH="$("${CTL}" health "${CFG}")"
for _ in $(seq 1 80); do
  ok=1
  for role in camera fc bev track map web; do
    if ! grep -qE "^${role} ready=1 " <<<"${HEALTH}"; then
      ok=0
      break
    fi
  done
  if [[ "${ok}" -eq 1 ]]; then
    for snap in snapshot_camera snapshot_fc snapshot_bev; do
      if ! grep -q "^${snap} valid=1" <<<"${HEALTH}"; then
        ok=0
        break
      fi
    done
  fi
  if [[ "${ok}" -eq 1 ]]; then
    break
  fi
  sleep 0.2
  HEALTH="$("${CTL}" health "${CFG}" 2>/dev/null || true)"
done

fail() {
  echo "parity validation failed: $1" >&2
  {
    echo "# MP Parity Report"
    echo
    echo "- Result: FAIL"
    echo "- Reason: $1"
    echo
    echo "## Health Dump"
    echo '```'
    echo "${HEALTH}"
    echo '```'
  } > "${OUT}"
  exit 1
}

for role in camera fc bev track map web; do
  if ! grep -qE "^${role} ready=1 " <<<"${HEALTH}"; then
    fail "role ${role} is not ready"
  fi
done

if ! grep -q "^sync_kpi samples=" <<<"${HEALTH}"; then
  fail "sync_kpi line missing"
fi
if ! grep -q "^watchdog " <<<"${HEALTH}"; then
  fail "watchdog line missing"
fi

for stage in stage_camera stage_fc stage_bev stage_track stage_map stage_web; do
  if ! grep -q "^${stage} " <<<"${HEALTH}"; then
    fail "${stage} line missing"
  fi
done

for snap in snapshot_camera snapshot_fc snapshot_bev; do
  if ! grep -q "^${snap} valid=1" <<<"${HEALTH}"; then
    fail "${snap} not valid"
  fi
done
if ! grep -q "^snapshot_map valid=" <<<"${HEALTH}"; then
  fail "snapshot_map line missing"
fi

if ! "${CTL}" pause-map "${CFG}" >/dev/null 2>&1; then
  fail "pause-map control failed"
fi
if ! "${CTL}" resume-map "${CFG}" >/dev/null 2>&1; then
  fail "resume-map control failed"
fi
H_RESUME="$("${CTL}" health "${CFG}")"

{
  echo "# MP Parity Report"
  echo
  echo "- Result: PASS"
  echo "- Config: ${CFG}"
  echo "- Calibration: ${CALIB}"
  echo "- Timestamp: $(date -Is)"
  echo
  echo "## Checks"
  echo "- All roles ready"
  echo "- sync_kpi line present"
  echo "- watchdog line present"
  echo "- stage_* KPI lines present"
  echo "- snapshot_* valid=1"
  echo "- pause-map/resume-map control command parity"
  echo
  echo "## Health (After Resume)"
  echo '```'
  echo "${H_RESUME}"
  echo '```'
} > "${OUT}"

echo "parity validation passed: ${OUT}"
