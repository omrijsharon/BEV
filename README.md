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
