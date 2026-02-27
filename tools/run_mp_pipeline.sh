#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/bev_mp_node"
CTL="${ROOT_DIR}/build/bev_ctl"
CFG="${1:-${ROOT_DIR}/config/config.yaml}"
CALIB="${2:-${ROOT_DIR}/config/camera_calibration.yaml}"

if [[ ! -x "${BIN}" || ! -x "${CTL}" ]]; then
  echo "missing binary: ${BIN} or ${CTL}"
  echo "build first: cmake -S . -B build && cmake --build build -j"
  exit 1
fi

pids=()
supervisor_pids=()
STOP_FILE="/tmp/bev_pipeline_stop.$$"
rm -f "${STOP_FILE}"
cleanup() {
  touch "${STOP_FILE}" >/dev/null 2>&1 || true
  "${CTL}" shutdown "${CFG}" >/dev/null 2>&1 || true
  sleep 0.2
  for p in "${supervisor_pids[@]:-}"; do
    kill "$p" >/dev/null 2>&1 || true
  done
  for p in "${pids[@]:-}"; do
    kill "$p" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

run_role() {
  local role="$1"
  while [[ ! -f "${STOP_FILE}" ]]; do
    "${BIN}" "${role}" "${CFG}" "${CALIB}" &
    local pid=$!
    echo "${pid}" > "/tmp/bev_${role}.pid"
    wait "${pid}" || true
    if [[ -f "${STOP_FILE}" ]]; then
      break
    fi
    echo "role ${role} exited; restarting in 1s"
    sleep 1
  done
}

run_role fc &
supervisor_pids+=($!)
sleep 0.2
run_role camera &
supervisor_pids+=($!)
sleep 0.2
run_role bev &
supervisor_pids+=($!)
sleep 0.2
run_role track &
supervisor_pids+=($!)
sleep 0.2
run_role map &
supervisor_pids+=($!)
sleep 0.2
run_role web &
supervisor_pids+=($!)

echo "waiting for control socket readiness..."
for _ in $(seq 1 100); do
  if "${CTL}" health "${CFG}" >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

echo "pipeline running. streams:"
echo "  http://localhost:8080/stream/main"
echo "  http://localhost:8080/stream/map"
echo "control examples:"
echo "  ./build/bev_ctl health"
echo "  ./build/bev_ctl pause-map"
echo "  ./build/bev_ctl resume-map"
echo "  ./build/bev_ctl shutdown"
echo "press Ctrl+C to stop."

wait
