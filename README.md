# BEV Project

Bird's eye view (BEV) drone vision pipeline for Raspberry Pi + Betaflight FC attitude integration.

## Build

```bash
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Notes

- `BetaflightMSP` sources are placed under `include/msp` and `src/msp`.
- By default, Arduino-dependent MSP source is not compiled in desktop builds (`BEV_BUILD_ARDUINO_MSP=OFF`).
- Core pipeline modules and tests are scaffolded and ready for milestone implementation.
- Camera mount verification runbook: `docs/mount_orientation_runbook.md`.
- Camera input supports `camera.source_mode: "v4l2" | "gstreamer"` in `config/config.yaml`.
- For `gstreamer`, configure `camera.gstreamer_pipeline`; runtime falls back to V4L2 if requested source fails.

## Multi-Process Roles

Role-based multiprocess node:

```bash
./build/bev_mp_node <role> [config_path] [calib_path]
```

Roles:
- `camera`
- `fc`
- `bev`
- `track`
- `map`
- `web`

Launcher script:

```bash
./tools/run_mp_pipeline.sh
```

Streams (when running):
- `http://localhost:8080/stream/main`
- `http://localhost:8080/stream/map`

Control examples:

```bash
./build/bev_ctl health
./build/bev_ctl pause-map
./build/bev_ctl resume-map
./build/bev_ctl reload-config
./build/bev_ctl shutdown
```

`health` output includes sync KPIs (`samples`, `stale`, `interp_ratio`, mean/max sync error).

IPC latency benchmark:

```bash
./build/bev_bench_ipc
./build/bev_bench_pipeline
```

Soak test harness (default 30 minutes):

```bash
./tests/soak/run_soak_pipeline.sh
```
