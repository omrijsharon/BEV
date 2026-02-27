# Multi-Process Low-Latency Conversion Plan (SHM + Ring Buffers)

## Summary
Convert the current mostly-serial single-process pipeline into a 6-process architecture optimized for low latency and higher FPS headroom using:
- Shared memory frame pools (zero-copy between stages)
- SPSC circular rings for stage-to-stage handoff
- `drop-old-keep-latest` backpressure policy
- Core pinning + RT scheduling on hot path processes
- Explicit frame-to-attitude timestamp synchronization (nearest/interpolated)

Chosen defaults (locked):
- Queue policy: `drop old, keep latest`
- Topology: `6 logical processes` (with 4-core deployment mapping)
- Scheduling: `core pinning + RT priority` (graceful fallback if unavailable)

## Target Architecture (6 Logical Processes)
1. `camera_proc` (camera ingest + undistort + timestamp)
- Inputs: camera driver / V4L2 / GStreamer
- Outputs:
  - undistorted frame in SHM `pool_cam`
  - `FrameDesc` into `Q_CAM_TO_BEV`

2. `fc_attitude_proc` (MSP ingest + attitude timestamping)
- Inputs: Betaflight MSP stream (UART)
- Outputs:
  - `AttitudeDesc` into `Q_FC_TO_BEV`
  - latest attitude snapshot

3. `bev_sync_proc` (time sync + BEV warp)
- Inputs: `Q_CAM_TO_BEV` frames + `Q_FC_TO_BEV` attitude samples
- Behavior:
  - for each frame timestamp, use nearest FC sample or interpolation between bracketing FC samples
  - compose mount transform + body/world transform, then warp to BEV
- Outputs:
  - BEV frame in SHM `pool_bev`
  - `BevFrameDesc` into `Q_BEV_TO_TRACK`

4. `track_vel_proc` (keypoint tracking + velocity + registration)
- Inputs: `Q_BEV_TO_TRACK` + BEV frame slots
- Outputs:
  - `TrackResultDesc` to `Q_TRACK_TO_MAP`
  - optional overlay desc to `Q_TRACK_TO_WEB`

5. `map_proc` (mini-map update + confidence/drift)
- Inputs: `Q_TRACK_TO_MAP` + BEV frame slots
- Outputs:
  - map frame in SHM `pool_map`
  - `MapFrameDesc` to `Q_MAP_TO_WEB`

6. `web_ctrl_proc` (MJPEG/web + health/control)
- Inputs: latest BEV/map descriptors
- Outputs: `/stream/main`, `/stream/map`, health endpoints, control commands

## SHM Design
### 1) SHM Segment A: Frame Pools
- `pool_cam` (undistorted camera frames)
- `pool_bev` (BEV frames)
- `pool_map` (map frames)
- Slot header fields:
  - `slot_id`, `generation`, `timestamp_ns`, `width`, `height`, `type`, `stride`, `state`, `refcount`

### 2) SHM Segment B: Ring Buffers (SPSC per edge)
- `Q_CAM_TO_BEV`
- `Q_FC_TO_BEV`
- `Q_BEV_TO_TRACK`
- `Q_TRACK_TO_MAP`
- `Q_TRACK_TO_WEB` (optional)
- `Q_MAP_TO_WEB`

Frame descriptor payload:
- `frame_slot_id`, `generation`, `timestamp_ns`, `sequence_id`

FC descriptor payload:
- `sample_id`, `timestamp_ns`
- `roll/pitch/yaw`, `quaternion` (quaternion preferred for interpolation)
- optional `R_body_world`

### 3) SHM Segment C: Latest-State Snapshots
- latest camera frame desc
- latest FC attitude desc
- latest BEV desc
- latest map desc
- health/status snapshot

## IPC Protocols
### Data Plane (high rate)
- SHM + SPSC binary descriptors (versioned, fixed-size)
- wakeup:
  - default: `eventfd`
  - fallback: bounded polling + nanosleep

### Control Plane (low rate)
- Unix domain socket (`AF_UNIX`) control messages:
  - `ReloadConfig`
  - `PauseMapUpdates`
  - `ResumeMapUpdates`
  - `SetLogLevel`
  - `RequestHealthSnapshot`

## Timestamp Sync Policy (Frame <-> FC)
Primary mode:
- Interpolate between FC samples that bracket frame timestamp.

Fallback mode:
- Nearest FC sample if bracket is unavailable.

Safety guards:
- `max_attitude_age_ms`
- `max_interp_gap_ms`
- `startup_min_fc_buffer`

Sync metrics:
- `sync_error_us`
- `stale_attitude_count`
- `interpolation_ratio`

## Circular Buffer / Race Strategy
1. Single writer / single reader ring per edge.
2. Descriptor carries `(slot_id, generation)`; consumer validates generation.
3. Refcount lifecycle protects slot reuse.
4. Overflow policy: drop oldest, never block producer.
5. Memory ordering: release on publish, acquire on consume.

## Scheduling / CPU Utilization Plan
4-core deployment mapping for 6 logical processes:
- Core 0: `camera_proc` (`SCHED_FIFO high`) + `fc_attitude_proc` (`SCHED_FIFO high-mid`)
- Core 1: `bev_sync_proc` (`SCHED_FIFO high`)
- Core 2: `track_vel_proc` (`SCHED_FIFO high-mid`)
- Core 3: `map_proc` (`SCHED_FIFO mid`) + `web_ctrl_proc` (`SCHED_OTHER normal`)

Additional:
- `mlockall(MCL_CURRENT|MCL_FUTURE)` for hot path processes
- pre-allocate all buffers and rings
- avoid dynamic allocation in per-frame loops
- fallback to non-RT scheduling if privileges are missing

## Data Flow and Contracts
1. `camera_proc`
- writes undistorted frame slot to `pool_cam`
- pushes `FrameDesc` to `Q_CAM_TO_BEV`

2. `fc_attitude_proc`
- pushes `AttitudeDesc` to `Q_FC_TO_BEV`
- updates latest snapshot

3. `bev_sync_proc`
- consumes frame desc
- resolves matching/interpolated attitude by timestamp
- computes `R_cam_world`
- writes BEV slot to `pool_bev`
- pushes `BevFrameDesc` to `Q_BEV_TO_TRACK`

4. `track_vel_proc`
- consumes `BevFrameDesc`
- computes tracking + velocity + registration
- pushes `TrackResultDesc` to `Q_TRACK_TO_MAP`

5. `map_proc`
- consumes `TrackResultDesc` + BEV slot
- updates minimap (unless paused)
- writes map slot to `pool_map`
- pushes `MapFrameDesc` to `Q_MAP_TO_WEB`

6. `web_ctrl_proc`
- serves latest BEV/map streams
- exposes queue depths, drop counters, sync KPIs

## Public Interfaces / Types to Add
- `IpcConfig`
  - `frame_pool_slots_*`
  - `ring_capacity_*`
  - `queue_policy`
  - `affinity.*`
  - `rt_priority.*`
  - sync policy params (`max_attitude_age_ms`, `max_interp_gap_ms`, `startup_min_fc_buffer`)
- `FrameDesc`
- `AttitudeDesc`
- `BevFrameDesc`
- `TrackResultDesc`
- `MapFrameDesc`
- `HealthSnapshot`
- `ControlMessage` union

## Migration Plan (Implementation Order)
1. Phase 1: IPC primitives (same binary)
- SHM frame pools + SPSC rings + descriptor contracts

2. Phase 2: Single-binary adapter boundaries
- camera adapter, FC adapter, BEV sync adapter, tracking/map adapters

3. Phase 3: Process split
- create 6 executables + supervisor launch

4. Phase 4: RT/affinity + telemetry
- queue depths, drops, per-stage latency, sync KPIs

5. Phase 5: High-load tuning (>15 FPS)
- ring sizes, stage budgets, drop thresholds, sync delay tuning

## Failure Modes and Recovery
- Producer crash:
  - downstream detects stale sequence timeout and raises health alert
- Ring overflow:
  - drop counters increase while latency remains bounded
- Slot mismatch `(slot_id,generation)`:
  - descriptor discarded + integrity counter increment
- FC stream gaps:
  - nearest fallback enabled; stale guard can reject warp/update
- Process restart:
  - supervisor restart + SHM reattach + protocol version checks

## Test Cases and Scenarios
1. IPC correctness
- wraparound, generation mismatch, refcount lifecycle

2. Sync correctness
- nearest/interpolated matching against synthetic timestamp timelines

3. Load behavior
- overload tracker and confirm bounded latency via drop-old policy

4. Recovery
- kill/restart individual process and verify continuity

5. Latency/FPS
- P50/P95/P99 per stage and end-to-end

6. Race safety
- high turnover stress under CPU contention

## Acceptance Criteria
- End-to-end latency (`camera ts -> velocity output`) improved by >=30% vs serial baseline
- Stable 15 FPS sustained with bounded queues
- No payload copies between processes for camera/BEV/map frames
- Clean degradation under overload (more drops, bounded latency)
- Timestamp alignment quality:
  - `P95 sync_error_us` within configured bound
  - stale fallback rate under configured threshold

## Assumptions and Defaults
- Linux target (Raspberry Pi OS / Ubuntu) with POSIX SHM + `eventfd`
- Single machine, local SHM only
- Drop-old policy accepted
- Existing CV algorithms unchanged initially; execution model first

## Suggestions (Performance to the Limit)
- [Suggestion] Prefer quaternion interpolation (slerp) over Euler interpolation for attitude sync stability.
- [Suggestion] Keep camera and FC clocks in one monotonic domain and timestamp at ingest boundary.
- [Suggestion] Add real-time telemetry gating: reduce web quality before impacting tracker latency.
- [Suggestion] Add adaptive FPS and optional tracking downscale path for thermal/headroom control.
- [Suggestion] Add periodic zero-copy audit to detect accidental frame copies in hot path.

## Milestones, Tasks, and Sub-Tasks (Checkbox + Finish Time)
Legend:
- Format: `- [ ] Task text (Finished: ____-__-__ __:__)`

### Milestone MP0: IPC Foundations
- [x] MP0-T1 Define binary descriptor structs (`BevFrameDesc`, `TrackResultDesc`, `MapFrameDesc`) and protocol versioning (Finished: 2026-02-26 23:44)
- [x] MP0-T1.a Add ABI-safe packing/alignment checks and static asserts (Finished: 2026-02-26 23:44)
- [x] MP0-T2 Implement SHM frame pool allocator with slot headers + refcounts (Finished: 2026-02-26 23:44)
- [x] MP0-T2.a Add generation counter semantics and integrity validation APIs (Finished: 2026-02-26 23:44)
- [x] MP0-T2.b Fix writer-slot lifecycle: release producer ownership on publish and only acquire writable slots from `Free` state (Finished: 2026-02-27 19:38)
- [x] MP0-T3 Implement SPSC ring buffer library for inter-stage descriptors (Finished: 2026-02-26 23:44)
- [x] MP0-T3.a Implement drop-old policy and overflow/drop counters (Finished: 2026-02-26 23:44)
- [x] MP0-T4 Implement `eventfd` wakeup path with polling fallback (Finished: 2026-02-26 23:44)
- [x] MP0-T5 Add `IpcConfig` schema + config loader + validation (Finished: 2026-02-26 23:44)

### Milestone MP1: Single-Binary Pipeline with IPC Boundaries
- [x] MP1-T1 Refactor serial flow to stage adapters with descriptor handoff (Finished: 2026-02-27 00:25)
- [x] MP1-T1.a Replace direct image handoff with SHM slot references (Finished: 2026-02-27 00:25)
- [x] MP1-T1.b Add camera/FC async adapter boundaries and independent rings (Finished: 2026-02-27 00:25)
- [x] MP1-T2 Integrate queue-depth/drop metrics and per-stage latency timing (Finished: 2026-02-27 22:45)
- [x] MP1-T2.a Add structured logs for continuity and backlog behavior (Finished: 2026-02-27 22:45)
- [x] MP1-T3 Implement latest snapshot block for web low-lag consumption (Finished: 2026-02-27 22:45)
- [x] MP1-T3.a Implement frame<->attitude timestamp matcher (nearest + interpolation) (Finished: 2026-02-27 00:25)
- [x] MP1-T4 Validate parity with current behavior (Finished: 2026-02-27 22:59)

### Milestone MP2: Process Split
Implementation note: currently realized as role-based processes in `bev_mp_node` (`camera|fc|bev|track|map|web`), launched as separate OS processes.
- [x] MP2-T1 Create `camera_proc` executable (capture, undistort, timestamp, publish `Q_CAM_TO_BEV`) (Finished: 2026-02-27 00:25)
- [x] MP2-T1.a Publish camera frame descriptors + latest camera snapshot (Finished: 2026-02-27 22:45)
- [x] MP2-T2 Create `fc_attitude_proc` executable (MSP read, timestamp, publish `Q_FC_TO_BEV`) (Finished: 2026-02-27 00:25)
- [x] MP2-T2.a Publish attitude descriptors + latest attitude snapshot (Finished: 2026-02-27 22:45)
- [x] MP2-T3 Create `bev_sync_proc` executable (timestamp match + BEV warp, publish `Q_BEV_TO_TRACK`) (Finished: 2026-02-27 00:25)
- [x] MP2-T3.a Implement interpolation/nearest fallback with staleness guards (Finished: 2026-02-27 00:25)
- [x] MP2-T4 Create `track_vel_proc` executable (tracking, velocity, registration) (Finished: 2026-02-27 00:25)
- [x] MP2-T4.a Publish `TrackResultDesc` into `Q_TRACK_TO_MAP` (and optional web queue) (Finished: 2026-02-27 00:25)
- [x] MP2-T5 Create `map_proc` executable (minimap update, drift/confidence) (Finished: 2026-02-27 00:25)
- [x] MP2-T5.a Publish `MapFrameDesc` into `Q_MAP_TO_WEB` and health snapshot (Finished: 2026-02-27 22:45)
- [x] MP2-T6 Create `web_ctrl_proc` executable (MJPEG serving, health/control endpoints) (Finished: 2026-02-27 00:25)
- [x] MP2-T6.a Consume latest BEV/map descriptors and serve stable streams (Finished: 2026-02-27 00:25)

### Milestone MP3: Control Plane and Supervision
Implementation note: control socket, health snapshot, pause/resume-map, shutdown, and launcher restart/readiness are implemented; robust SHM reattach/rehydration after producer recreation remains pending.
- [x] MP3-T1 Implement Unix-domain control protocol (`ReloadConfig`, pause/resume map, health request) (Finished: 2026-02-27 10:05)
- [x] MP3-T2 Implement process supervisor (systemd units or launcher script with restart policy) (Finished: 2026-02-27 00:25)
- [x] MP3-T2.a Add startup ordering and readiness checks for SHM/rings (Finished: 2026-02-27 00:25)
- [x] MP3-T3 Implement graceful shutdown and reattach-on-restart behavior (Finished: 2026-02-27 06:26)
- [x] MP3-T3.a Reset/rehydrate logic on protocol mismatch or stale SHM generation (Finished: 2026-02-27 06:26)
- [x] MP3-T3.b Make `SharedState::create` non-destructive for already-existing compatible segments (avoid health/ready reset on role restart) (Finished: 2026-02-27 19:40)
- [x] MP3-T3.c Harden no-camera startup path in `camera_proc` (`v4l2` strict open + deterministic synthetic fallback trigger) (Finished: 2026-02-27 19:40)
- [x] MP3-T3.d Reset control flags on pipeline bootstrap (`shutdown/pause/protocol_mismatch`) to prevent stale shutdown latch after restart (Finished: 2026-02-27 19:45)
- [x] MP3-T3.e Reset role health + sync KPI counters on pipeline bootstrap to avoid stale `t=0` health snapshots (Finished: 2026-02-27 19:46)

### Milestone MP4: CPU Scheduling and Memory Determinism
Implementation note: role affinity + RT + `mlockall` are implemented; hot-loop allocation guards now include mat reuse and in-place keypoint buffers in tracker loop.
- [x] MP4-T1 Implement CPU affinity policy for 6 logical processes on 4 cores (Finished: 2026-02-27 06:26)
- [x] MP4-T2 Implement RT scheduling policy (`SCHED_FIFO`) with fallback path (Finished: 2026-02-27 06:26)
- [x] MP4-T2.a Add capability/privilege checks + warnings when RT unavailable (Finished: 2026-02-27 06:26)
- [x] MP4-T3 Implement `mlockall` and startup pre-allocation of hot-path buffers (Finished: 2026-02-27 06:26)
- [x] MP4-T4 Remove/guard dynamic allocations from per-frame hot loops (Finished: 2026-02-27 18:09)

### Milestone MP5: Validation, Benchmarks, and Hardening
Implementation note: soak-test harness is implemented (`tests/soak/run_soak_pipeline.sh`) and smoke-validated; run with default 1800s for full 30+ minute qualification.
- [x] MP5-T1 Add IPC correctness tests (ring wraparound, generation mismatch, refcount lifecycle) (Finished: 2026-02-26 23:44)
- [x] MP5-T2 Add overload tests to verify latency-bounded drop-old behavior (Finished: 2026-02-27 10:05)
- [x] MP5-T2.a Add sync-correctness tests for nearest/interpolated FC matching (Finished: 2026-02-27 00:25)
- [x] MP5-T3 Add crash/restart resilience tests per process (Finished: 2026-02-27 17:44)
- [x] MP5-T4 Add benchmark suite for stage latency + end-to-end P50/P95/P99 + FPS (Finished: 2026-02-27 17:44)
- [x] MP5-T4.a Compare against serial baseline and generate regression report artifact (Finished: 2026-02-27 17:44)
- [x] MP5-T5 Add long-run soak test (30+ min) for queue stability and leak detection (Finished: 2026-02-27 18:09)
- [x] MP5-T5.a Improve soak startup gate to wait for non-fallback control health before sampling (Finished: 2026-02-27 19:43)
- [x] MP5-T6 Final tuning pass to prepare >15 FPS target envelope (Finished: 2026-02-27 18:09)
- [x] MP5-T6.a Add sync KPI dashboard (`sync_error_us`, stale rate, interpolation ratio) (Finished: 2026-02-27 18:09)

### Milestone MP6: Extreme Throughput Optimization
- [x] MP6-T1 Add adaptive frame-rate governor (camera FPS vs tracker backlog) (Finished: 2026-02-27 23:10)
- [x] MP6-T2 Add optional downscale-for-tracking path with full-res BEV/map retention (Finished: 2026-02-27 23:38)
- [x] MP6-T3 Add zero-copy web path where possible (direct JPEG from SHM slot lifecycle) (Finished: 2026-02-27 23:46)
- [x] MP6-T4 Add cache/memory profiling and hot-loop micro-optimizations (Finished: 2026-02-27 23:46)
- [x] MP6-T5 Add CPU thermal throttling observability + mitigation policy (Finished: 2026-02-27 23:46)

## Priority Order (Foundation First)
1. MP0 (`IPC Foundations`)
2. MP1 (`Single-Binary IPC Boundaries`)
3. MP2 (`Process Split`)
4. MP3 (`Control + Supervision`)
5. MP4 (`Scheduling + Determinism`)
6. MP5 (`Validation + Tuning`)
7. MP6 (`Extreme Throughput Optimization`)

Dependency logic:
1. MP1 depends on MP0.
2. MP2 depends on MP1.
3. MP3 depends on MP2.
4. MP4 depends on MP2 (and partially MP3 for startup order).
5. MP5 depends on MP2/MP3/MP4.
6. MP6 depends on MP5 baseline passing.
