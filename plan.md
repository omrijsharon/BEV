# BEV Drone Vision Project Plan

## 1) Project Goal
Build a real-time, efficient BEV (bird's eye view) pipeline on Raspberry Pi that:
1. Estimates drone ego translation from a downward-facing camera.
2. Aligns consecutive frames despite rotation and translation.
3. Detects motion from aligned frame differences.
4. Builds a continuously updated stitched mini-map.

Target stack and defaults:
- Language/runtime: C++ + OpenCV
- Target performance: 15 FPS at 640x480
- Map mode (v1): Pixel-space mosaic (not metric geo-map yet)
- Sync mode: Timestamp nearest-sample attitude sync
- Alignment refinement: Homography (`cv::findHomography` + `cv::warpPerspective`)
- Web streaming: MJPEG over HTTP

## 2) Q&A (Your Questions)
### Q1: What is the alignment called in computer vision?
It is called **image registration** (or frame alignment / motion compensation).

### Q2: After tracking keypoints, what OpenCV function should we use to align images?
Use:
1. `cv::findHomography(...)` with `RANSAC` on matched keypoints.
2. `cv::warpPerspective(...)` to warp one frame onto the other (or onto map frame).

Recommended sequence:
1. IMU-based BEV pre-warp.
2. Keypoint tracking/matching.
3. Homography refinement with RANSAC.
4. Final warp for precise alignment.

## 3) System Architecture
### 3.1 Modules
1. `msp_bridge`
- Read FC attitude from `BetaflightMSP` over UART.
- Convert units: roll/pitch deci-degrees -> radians, yaw degrees -> radians.
- Publish timestamped attitude samples.

2. `camera_ingest`
- Capture frames (640x480).
- Timestamp each frame.
- Undistort with known intrinsics and distortion coefficients.

3. `bev_warp`
- Build rotation matrix from attitude.
- Compute orientation BEV transform.
- Apply coarse BEV warp (`warpPerspective`).

4. `feature_tracker`
- Track configurable number of keypoints.
- Efficient local template matching:
  - Coarse pass with stride 2 inside coarse crop.
  - Fine pass with stride 1 inside fine crop.
- Convert crop-local match position to global BEV pixel coordinates.
- Predict next crop center with velocity model and cap using `max_distance_from_keypoint_center`.

5. `registration_refiner`
- Estimate frame-to-frame homography from tracked points with RANSAC.
- Reject outliers by reprojection error.
- Produce final alignment transform.

6. `motion_detector`
- Signed grayscale difference between aligned frames.
- Deadzone threshold around zero.
- Overlay:
  - Negative motion in blue
  - Positive motion in red
  - Shared alpha transparency

7. `minimap_builder`
- Maintain global transform chain and stitched canvas.
- Expand canvas as needed.
- Blend incoming frames into mini-map.

8. `webapp_stream`
- Serve MJPEG stream for:
  - Main aligned BEV + motion overlay
  - Mini-map
- Show runtime status (FPS, keypoints, inliers).

9. `config_manager`
- Load and validate configuration values at startup.

### 3.2 Proposed Repository Structure
```text
BEV/
├── CMakeLists.txt
├── plan.md
├── README.md
├── config/
│   ├── config.yaml
│   └── camera_calibration.yaml
├── include/
│   ├── core/
│   │   ├── types.hpp                 # AttitudeSample, FramePacket, TrackPoint, RegistrationResult
│   │   ├── config.hpp                # Config structs and validation API
│   │   ├── math_utils.hpp            # Rotation/transform helpers
│   │   ├── time_utils.hpp            # Timestamp helpers
│   │   └── telemetry.hpp             # Runtime counters and health status
│   ├── msp/
│   │   ├── msp_bridge.hpp            # BetaflightMSP wrapper + timestamped attitude buffering
│   │   └── betaflight_msp_adapter.hpp
│   ├── camera/
│   │   ├── camera_ingest.hpp         # Capture + undistort interface
│   │   └── camera_calibration.hpp
│   ├── bev/
│   │   └── bev_warp.hpp              # IMU-based coarse BEV warp
│   ├── tracking/
│   │   ├── keypoint_initializer.hpp
│   │   ├── template_tracker.hpp      # Coarse/fine template matching
│   │   └── track_manager.hpp         # Track lifecycle, velocity prediction, reinit
│   ├── registration/
│   │   └── registration_refiner.hpp  # Homography + RANSAC refinement
│   ├── motion/
│   │   └── motion_detector.hpp       # Signed diff + threshold + color overlay
│   ├── map/
│   │   └── minimap_builder.hpp       # Stitching, blending, canvas expansion
│   └── web/
│       ├── mjpeg_server.hpp          # HTTP MJPEG endpoints
│       └── dashboard_renderer.hpp    # HUD and compositing helpers
├── src/
│   ├── main.cpp                      # App orchestration and runtime loop
│   ├── core/
│   │   ├── config.cpp
│   │   ├── math_utils.cpp
│   │   ├── time_utils.cpp
│   │   └── telemetry.cpp
│   ├── msp/
│   │   ├── msp_bridge.cpp
│   │   └── betaflight_msp_adapter.cpp
│   ├── camera/
│   │   ├── camera_ingest.cpp
│   │   └── camera_calibration.cpp
│   ├── bev/
│   │   └── bev_warp.cpp
│   ├── tracking/
│   │   ├── keypoint_initializer.cpp
│   │   ├── template_tracker.cpp
│   │   └── track_manager.cpp
│   ├── registration/
│   │   └── registration_refiner.cpp
│   ├── motion/
│   │   └── motion_detector.cpp
│   ├── map/
│   │   └── minimap_builder.cpp
│   └── web/
│       ├── mjpeg_server.cpp
│       └── dashboard_renderer.cpp
├── third_party/
│   └── (optional external deps if needed)
├── tests/
│   ├── CMakeLists.txt
│   ├── unit/
│   │   ├── test_math_utils.cpp
│   │   ├── test_config_validation.cpp
│   │   ├── test_tracker_coordinates.cpp
│   │   └── test_motion_threshold.cpp
│   └── integration/
│       ├── test_bev_alignment.cpp
│       ├── test_homography_refine.cpp
│       └── test_minimap_stitching.cpp
├── tools/
│   ├── replay/
│   │   └── replay_log.cpp            # Offline frame+attitude replay utility
│   └── profiling/
│       └── profile_pipeline.sh
├── data/
│   ├── logs/
│   └── recordings/
└── web/
    ├── static/
    │   ├── index.html
    │   ├── app.js
    │   └── styles.css
    └── README.md
```

## 4) Configuration (v1)
Use `config.yaml` with at least:
- `tracking.num_keypoints`
- `tracking.template_size`
- `tracking.coarse_crop_size`
- `tracking.fine_crop_size`
- `tracking.max_distance_from_keypoint_center`
- `tracking.min_valid_tracks`
- `tracking.ransac_reproj_threshold_px`
- `motion.alpha`
- `motion.threshold`
- `camera.width`
- `camera.height`
- `camera.fps`
- `map.initial_width`
- `map.initial_height`
- `map.expand_margin_px`

## 5) Data Flow
1. Capture frame + timestamp.
2. Fetch nearest/interpolated attitude sample for that timestamp.
3. Undistort frame.
4. IMU-based BEV coarse warp.
5. Track keypoints (coarse-to-fine local matching).
6. Estimate homography refinement (RANSAC).
7. Apply final frame alignment.
8. Compute motion mask and overlay.
9. Update stitched mini-map.
10. Publish web streams and telemetry.

## 6) Clarification Questions (Open)
1. Camera shutter type and sensor mode (rolling/global)?
2. Typical and worst-case yaw rate expected during operation?
3. Typical altitude band for flights in this project?
4. Preferred camera exposure mode (fixed/manual vs auto) for motion-diff stability?
5. Should map history be unlimited, or should we enforce memory bounds after N MB?

## 7) Suggestions
1. Add CLAHE preprocessing on grayscale before keypoint/template matching in low contrast scenes.
2. Add periodic keyframe re-registration to reduce long-term stitching drift.
3. Log transforms and confidence metrics for offline replay/debug.
4. Add fallback detector (e.g., ORB) only when local template tracking degrades.
5. Consider fixed exposure for stable differencing (auto exposure may create false motion).

## 8) Milestones, Tasks, Sub-Tasks (with Checkboxes + Finish Time Placeholder)
Legend:
- Format: `- [ ] Task text (Finished: ____-__-__ __:__)`

### Milestone M0: Foundation and Project Skeleton
- [x] M0-T1 Create C++ project skeleton and build system (Finished: 2026-02-26 17:25)
- [ ] M0-T1.a Add OpenCV dependency and verify compile/link on Raspberry Pi (WSL Ubuntu build+ctest verified at 2026-02-26 19:34; Raspberry Pi verification pending) (Finished: ____-__-__ __:__)
- [x] M0-T1.b Define module folders (`msp_bridge`, `camera_ingest`, `bev_warp`, `tracker`, `registration`, `motion`, `map`, `web`, `config`) (Finished: 2026-02-26 17:25)
- [x] M0-T2 Define core data types and interfaces (`AttitudeSample`, `FramePacket`, `TrackPoint`, `RegistrationResult`) (Finished: 2026-02-26 17:25)
- [x] M0-T2.a Add unit conversion utilities (deci-deg/deg to rad) (Finished: 2026-02-26 17:25)
- [x] M0-T3 Implement config loader (`config.yaml`) and validation checks (Finished: 2026-02-26 17:25)
- [x] M0-T3.a Add defaults and range validation for all critical params (Finished: 2026-02-26 17:25)
- [x] M0-T4 Define logging and telemetry counters (FPS, tracks, inliers, latency) (Finished: 2026-02-26 17:25)
- [x] M0-T5 Move `BetaflightMSP` sources into project tree (`include/msp`, `src/msp`) (Finished: 2026-02-26 17:25)

### Milestone M1: Attitude Sync + Coarse BEV Warp + Web Preview
- [x] M1-T1 Implement MSP attitude reader wrapper around `BetaflightMSP` (Finished: 2026-02-26 19:17)
- [x] M1-T1.a Add timestamped ring buffer for attitude samples (Finished: 2026-02-26 19:14)
- [x] M1-T1.b Implement nearest/interpolated sample lookup by frame timestamp (Finished: 2026-02-26 19:14)
- [x] M1-T2 Implement camera capture + undistortion pipeline (Finished: 2026-02-26 19:14)
- [ ] M1-T2.a Validate intrinsics/distortion load and undistort output sanity (Calibration file load validated by unit tests; on-device camera validation pending) (Finished: ____-__-__ __:__)
- [x] M1-T2.b Add configurable camera source mode (`v4l2` | `gstreamer`) in config (Finished: 2026-02-26 22:34)
- [x] M1-T2.c Add configurable `camera.gstreamer_pipeline` string in config and loader validation (Finished: 2026-02-26 22:34)
- [x] M1-T2.d Implement camera open via `cv::VideoCapture(pipeline, cv::CAP_GSTREAMER)` with fallback to `v4l2` (Finished: 2026-02-26 22:34)
- [x] M1-T2.e Add Raspberry Pi default GStreamer pipeline template and runtime logging of selected source path (Finished: 2026-02-26 22:34)
- [x] M1-T3 Implement IMU-based BEV coarse warp using rotation matrix + `warpPerspective` (Finished: 2026-02-26 19:14)
- [ ] M1-T3.a Validate roll/pitch/yaw sign convention and axis mapping (Math path tested; FC+camera real-flight sign validation pending) (Finished: ____-__-__ __:__)
- [x] M1-T3.b Add `camera_mount` extrinsic parameters to config (`roll_deg`, `pitch_deg`, `yaw_deg`, `negate_yaw`) and validation rules (Finished: 2026-02-26 22:17)
- [x] M1-T3.c Compose orientation with mount extrinsics: `R_cam_world = R_cam_body * R_body_world` and use it in BEV homography (Finished: 2026-02-26 22:17)
- [x] M1-T3.d Add runtime A/B check for sensor-plane rotation (`roll_deg = +90` vs `-90`) using alignment score and log recommended sign (Finished: 2026-02-26 22:17)
- [x] M1-T3.e Add mount-orientation verification runbook (small yaw/pitch/roll maneuvers + expected image motion checks) (Finished: 2026-02-26 22:17)
- [x] M1-T4 Implement MJPEG web stream for coarse BEV frame (Socket-based MJPEG endpoint + app wiring on `/stream/main`) (Finished: 2026-02-26 19:38)
- [x] M1-T4.a Add simple status overlay (FPS + MSP freshness) (Finished: 2026-02-26 19:17)

### Milestone M2: Keypoint Tracking and Fine Registration
- [x] M2-T1 Implement keypoint initialization strategy (grid or Shi-Tomasi) on BEV frame (Finished: 2026-02-26 19:19)
- [x] M2-T2 Implement coarse template match (stride 2) in local crop (Finished: 2026-02-26 19:20)
- [x] M2-T2.a Convert coarse-local coordinates back to global frame coordinates (Finished: 2026-02-26 19:20)
- [x] M2-T3 Implement fine template match (stride 1) in tighter crop (Finished: 2026-02-26 19:20)
- [x] M2-T3.a Add confidence score and failure rejection per keypoint (Finished: 2026-02-26 19:20)
- [x] M2-T4 Add velocity prediction for crop center (`position + dt * velocity`) with max cap clamp (Finished: 2026-02-26 19:19)
- [x] M2-T4.b Implement end-to-end live pixel-velocity estimator in runtime loop (persistent frame-to-frame tracking state) (Finished: 2026-02-27 20:23)
- [x] M2-T4.c Update per-keypoint and aggregate frame velocity each frame from tracked correspondences (Finished: 2026-02-27 20:23)
- [x] M2-T4.d Feed estimated velocity back into next-frame coarse/fine search-center placement in live pipeline (Finished: 2026-02-27 20:23)
- [x] M2-T5 Implement homography refinement with `findHomography(RANSAC)` (Finished: 2026-02-26 19:19)
- [x] M2-T5.a Add inlier ratio and reprojection RMSE gates (Finished: 2026-02-26 19:19)
- [x] M2-T6 Implement fallback/recovery when valid tracks drop below threshold (Finished: 2026-02-26 19:50)
- [x] M2-T6.a Reinitialize keypoints and temporarily pause map updates when confidence is low (Finished: 2026-02-26 19:50)

### Milestone M3: Motion Detection Overlay
- [x] M3-T1 Implement aligned grayscale signed frame differencing (Finished: 2026-02-26 19:22)
- [x] M3-T2 Apply deadzone threshold (`[-threshold, +threshold] -> 0`) (Finished: 2026-02-26 19:22)
- [x] M3-T3 Render blue negative / red positive mask with configurable alpha (Finished: 2026-02-26 19:22)
- [x] M3-T3.a Optimize mask generation to minimize allocations per frame (In-place/reusable-buffer APIs added in `motion_detector`) (Finished: 2026-02-26 19:36)
- [x] M3-T4 Add web toggle controls for motion overlay visualization (Dashboard UI controls added in `web/static`) (Finished: 2026-02-26 19:36)

### Milestone M4: Mini-Map Stitching
- [x] M4-T1 Define global map coordinate frame and cumulative transform handling (Finished: 2026-02-26 19:24)
- [x] M4-T2 Implement frame insertion into map via accumulated homography (Finished: 2026-02-26 19:24)
- [x] M4-T2.a Blend new data with existing map pixels (Finished: 2026-02-26 19:24)
- [x] M4-T3 Implement dynamic canvas expansion when boundaries are reached (Finished: 2026-02-26 19:24)
- [x] M4-T3.a Maintain transform consistency after expansion/re-centering (Finished: 2026-02-26 19:24)
- [x] M4-T4 Publish mini-map MJPEG stream endpoint (Finished: 2026-02-26 19:50)
- [x] M4-T4.a Add drift/confidence indicator to mini-map view (Finished: 2026-02-26 19:50)

### Milestone M5: Robustness, Profiling, and Validation
- [x] M5-T1 Add unit tests for math, transforms, thresholding, and coordinate conversion (Finished: 2026-02-26 17:25)
- [x] M5-T2 Add integration tests on recorded flight sequences (Added sequence registration replay, motion replay, and minimap replay integration tests; verified in WSL with `ctest`) (Finished: 2026-02-26 20:07)
- [x] M5-T2.a Measure registration error, inlier ratio, and motion false positives (Metrics emitted by integration tests to `data/logs/metrics_sequence_registration.txt` and `data/logs/metrics_motion_replay.txt`) (Finished: 2026-02-26 20:17)
- [x] M5-T3 Profile CPU/memory and optimize hot paths for 15 FPS sustained (Hot-path profiling script + runtime stage latency/queue KPIs + grayscale web path optimization) (Finished: 2026-02-27 23:40)
- [x] M5-T3.a Add pre-allocation and frame pipeline buffering checks (Shared-memory ring/pool buffering checks and stage queue-depth/drop visibility in health output) (Finished: 2026-02-27 23:42)
- [x] M5-T4 Add runtime health watchdogs (MSP stale data, tracker failure streaks, map instability) (Shared-state watchdog counters/alerts published by runtime and surfaced via `bev_ctl health`) (Finished: 2026-02-27 23:56)
- [x] M5-T5 Final acceptance run (10+ min continuous) and report (Passed 600s soak: `docs/soak_report_10min_2026-02-28_00-06-23.log`, no health_unavailable, stale=0, watchdog alerts=0 at end) (Finished: 2026-02-28 00:18)

## 9) Task Priority (Foundation First)
Priority order (highest to lowest):
1. **P0 Foundation**: All M0 tasks (project skeleton, interfaces, config, telemetry).
2. **P1 Core Input/Geometry**: M1 tasks (MSP sync, camera undistort, coarse BEV warp, basic web preview).
3. **P2 Alignment Core**: M2 tasks (tracking, efficient matching, homography refinement, recovery).
4. **P3 Motion Layer**: M3 tasks (frame difference mask and overlay UI).
5. **P4 Mapping Layer**: M4 tasks (stitching, expanding canvas, map streaming).
6. **P5 Hardening**: M5 tasks (testing, profiling, watchdogs, final validation).

Dependency logic:
1. M2 depends on M1 (needs stable BEV input and sync).
2. M3 depends on M2 (needs reliable aligned frames).
3. M4 depends on M2 (needs robust transforms and tracking).
4. M5 depends on all prior milestones.

## 10) Acceptance Criteria (v1)
1. Real-time pipeline reaches 15 FPS at 640x480 on target Pi.
2. Aligned consecutive BEV frames are visually stable and quantitatively consistent.
3. Motion overlay highlights meaningful changes with threshold control.
4. Mini-map grows and updates continuously without frequent catastrophic jumps.
5. Recovery path works when tracking quality drops (automatic re-initialization).
