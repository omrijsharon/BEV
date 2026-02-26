# Multi-Process Low-Latency Conversion Plan (SHM + Ring Buffers)

## Summary
Convert the current mostly-serial single-process pipeline into a 4-process architecture optimized for low latency and higher FPS headroom using:
- Shared memory frame pools (zero-copy between stages),
- SPSC circular rings for stage-to-stage handoff,
- "drop-old-keep-latest" backpressure policy,
- Core pinning + RT scheduling on hot path processes.

Chosen defaults (locked):
- Queue policy: `drop old, keep latest`
- Topology: `4 processes`
- Scheduling: `core pinning + RT priority` (with graceful fallback)

## Target Architecture (4 Processes)
1. `bev_capture_proc` (Camera + Undistort + BEV warp + attitude sync)
- Inputs: camera, MSP attitude
- Outputs:
  - BEV frame in SHM frame pool
  - metadata descriptor pushed to ring `Q_BEV_TO_TRACK`

2. `track_vel_proc` (Keypoint tracking + velocity + registration)
- Inputs: descriptors from `Q_BEV_TO_TRACK`, BEV frames by SHM index
- Outputs:
  - tracking/velocity/registration result descriptor to `Q_TRACK_TO_MAP`
  - optional BEV overlay publish descriptor to `Q_TRACK_TO_WEB`

3. `map_proc` (Mini-map update + drift/confidence computation)
- Inputs: descriptors from `Q_TRACK_TO_MAP`, BEV frames by SHM index
- Outputs:
  - latest map frame descriptor to `Q_MAP_TO_WEB`
  - map metrics to control channel

4. `web_ctrl_proc` (MJPEG/web + health/control)
- Inputs: latest BEV/map descriptors from rings
- Outputs: `/stream/main`, `/stream/map`, health endpoints, control commands (pause/restart/config reload)

## SHM Design
### 1) SHM Segment A: Frame Pool
- Fixed-size slot array for BEV frames (and optional raw/undistorted if needed later).
- Slot layout:
  - header: `slot_id`, `generation`, `timestamp_ns`, `width`, `height`, `type`, `stride`, `state`
  - payload: aligned pixel bytes
  - atomic refcount
- Suggested size:
  - start with `N=32` slots for 640x480 GRAY/BGR mix
  - configurable via `ipc.frame_pool_slots`

### 2) SHM Segment B: Ring Buffers (SPSC per edge)
- `Q_BEV_TO_TRACK`
- `Q_TRACK_TO_MAP`
- `Q_TRACK_TO_WEB` (optional)
- `Q_MAP_TO_WEB`
- Descriptor payload (no image copy):
  - `frame_slot_id`, `generation`, `timestamp_ns`, `sequence_id`
  - stage-specific metadata (velocity, confidence, inlier ratio, homography id)

### 3) SHM Segment C: Latest-State Snapshot (optional fast path)
- Single latest BEV descriptor
- Single latest map descriptor
- Last health/status snapshot
- Used by web to avoid lagging behind deep queues

## IPC Protocols
### Data Plane (high rate)
- SHM + SPSC ring descriptor protocol (binary fixed structs, versioned)
- Notification:
  - default: `eventfd` (Linux) for wakeup per ring
  - fallback: periodic polling with nanosleep if `eventfd` unavailable

### Control Plane (low rate)
- Unix domain socket (`AF_UNIX`, datagram or stream) with small versioned messages:
  - `ReloadConfig`
  - `PauseMapUpdates`
  - `ResumeMapUpdates`
  - `SetLogLevel`
  - `RequestHealthSnapshot`
- Encoding: fixed binary structs (no JSON in hot path)

## Circular Buffer / Race Condition Strategy
1. Single writer / single reader ring per pipeline edge (lock-free atomics).
2. Descriptor includes `(slot_id, generation)`:
- consumer verifies generation matches slot header before read.
3. Refcount lifecycle:
- producer acquires free slot -> writes -> pushes descriptor
- each consumer increments/decrements refcount around use
- slot reusable only when refcount returns to zero
4. Overflow policy:
- producer advances tail (drop oldest descriptor) and counts drop metrics
- never blocks hot path producer
5. Memory ordering:
- release store on write publish, acquire load on consume

## Scheduling / CPU Utilization Plan
Default affinity (Pi 4-core baseline):
- Core 0: `bev_capture_proc` (`SCHED_FIFO` high)
- Core 1: `track_vel_proc` (`SCHED_FIFO` high-mid)
- Core 2: `map_proc` (`SCHED_FIFO` mid)
- Core 3: `web_ctrl_proc` (`SCHED_OTHER` normal)

Additional:
- `mlockall(MCL_CURRENT|MCL_FUTURE)` for hot-path processes
- pre-allocate buffers at startup
- avoid dynamic allocations inside frame loop
- if RT privileges unavailable, fallback to normal scheduler + affinity only

## Data Flow and Contracts
1. `bev_capture_proc`
- writes BEV frame slot
- pushes `BevFrameDesc` to `Q_BEV_TO_TRACK`
- updates "latest BEV" snapshot

2. `track_vel_proc`
- consumes `BevFrameDesc`
- performs persistent tracking + velocity update
- outputs `TrackResultDesc`:
  - `frame_slot_id`, `sequence_id`
  - `vx_px_s`, `vy_px_s`, `omega_rad_s`
  - `inlier_ratio`, `valid_track_count`, `confidence`
  - registration transform id/reference
- pushes to `Q_TRACK_TO_MAP`
- optionally publishes overlay frame descriptor to web queue

3. `map_proc`
- consumes `TrackResultDesc` + referenced BEV slot
- updates minimap if `map_paused==false`
- emits `MapFrameDesc` + map metrics:
  - `coverage_ratio`, `drift_px`, `map_confidence`
- pushes to `Q_MAP_TO_WEB`

4. `web_ctrl_proc`
- always serves latest available BEV/map
- exposes health counters and queue depths

## Public Interfaces / Types to Add
- `IpcConfig`:
  - `frame_pool_slots`
  - `ring_capacity_*`
  - `queue_policy`
  - `affinity.*`
  - `rt_priority.*`
- `BevFrameDesc` (data plane descriptor)
- `TrackResultDesc`
- `MapFrameDesc`
- `HealthSnapshot`
- `ControlMessage` union (control plane)

## Migration Plan (Implementation Order)
1. Phase 1: Introduce IPC primitives in same binary (no process split yet)
- implement SHM frame pool + SPSC ring library
- replace direct stage calls with descriptor passing abstractions

2. Phase 2: Split into 4 executables
- `bev_capture_proc`, `track_vel_proc`, `map_proc`, `web_ctrl_proc`
- launch via supervisor script/systemd

3. Phase 3: Enable RT/affinity + telemetry
- queue depth, drops, per-stage latency, end-to-end latency

4. Phase 4: Performance tuning for >15 FPS
- ring capacities, frame pool sizing, stage budgets, drop thresholds

## Failure Modes and Recovery
- Producer crash:
  - consumer detects stale sequence timeout -> raise health alert
- Ring overflow:
  - drop counters increase; no blocking
- Slot corruption mismatch `(slot_id,generation)`:
  - discard descriptor; increment integrity error counter
- Process restart:
  - supervisor restarts process
  - reattach to SHM; reset rings if protocol version mismatch
- Backpressure storms:
  - temporary map pause and lower web refresh quality first (protect tracking latency)

## Test Cases and Scenarios
1. IPC correctness
- SPSC ring wraparound, generation mismatch checks, refcount lifecycle

2. Load behavior
- forced overload on tracker stage; verify drop-old policy keeps low latency

3. Recovery
- kill/restart one process; verify supervisor recovery and stream continuity

4. Latency/FPS
- measure per-stage and end-to-end latency percentiles (P50/P95/P99)
- verify improvement versus serial baseline

5. Determinism
- replay same synthetic sequence; verify stable output metrics across runs

6. Race safety
- stress with high ring turnover and forced CPU contention

## Acceptance Criteria
- End-to-end latency (BEV frame timestamp -> velocity result) reduced vs current serial baseline by >=30%
- Stable operation at 15 FPS under sustained run with no unbounded queue growth
- No frame-copy between processes for BEV/map payloads (descriptor-only transfer)
- Clean degradation under overload (drops increase, latency bounded)
- Web streams remain responsive while tracker/map remain prioritized

## Assumptions and Defaults
- Linux target (Raspberry Pi OS / Ubuntu) with POSIX SHM and `eventfd`
- Shared memory is local-host only (no distributed processing)
- Drop-old policy is acceptable for control/UX goals
- Single machine, multi-process, no GPU offload in this phase
- Existing CV algorithms remain unchanged initially; this plan focuses on execution architecture

## Milestones, Tasks, and Sub-Tasks (Checkbox + Finish Time)
Legend:
- Format: `- [ ] Task text (Finished: ____-__-__ __:__)`

### Milestone MP0: IPC Foundations
- [ ] MP0-T1 Define binary descriptor structs (`BevFrameDesc`, `TrackResultDesc`, `MapFrameDesc`) and protocol versioning (Finished: ____-__-__ __:__)
- [ ] MP0-T1.a Add ABI-safe packing/alignment checks and static asserts (Finished: ____-__-__ __:__)
- [ ] MP0-T2 Implement SHM frame pool allocator with slot headers + refcounts (Finished: ____-__-__ __:__)
- [ ] MP0-T2.a Add generation counter semantics and integrity validation APIs (Finished: ____-__-__ __:__)
- [ ] MP0-T3 Implement SPSC ring buffer library for inter-stage descriptors (Finished: ____-__-__ __:__)
- [ ] MP0-T3.a Implement drop-old policy and overflow/drop counters (Finished: ____-__-__ __:__)
- [ ] MP0-T4 Implement `eventfd` wakeup path with polling fallback (Finished: ____-__-__ __:__)
- [ ] MP0-T5 Add `IpcConfig` schema + config loader + validation (Finished: ____-__-__ __:__)

### Milestone MP1: Single-Binary Pipeline with IPC Boundaries
- [ ] MP1-T1 Refactor current serial flow to stage adapters using descriptor handoff (Finished: ____-__-__ __:__)
- [ ] MP1-T1.a Replace direct image handoff with SHM slot references (Finished: ____-__-__ __:__)
- [ ] MP1-T2 Integrate queue-depth/drop metrics and per-stage latency timing (Finished: ____-__-__ __:__)
- [ ] MP1-T2.a Add structured logs for sequence continuity and backlog behavior (Finished: ____-__-__ __:__)
- [ ] MP1-T3 Implement latest-frame snapshot block for web low-lag consumption (Finished: ____-__-__ __:__)
- [ ] MP1-T4 Validate functional parity with current single-process behavior (Finished: ____-__-__ __:__)

### Milestone MP2: Process Split
- [ ] MP2-T1 Create `bev_capture_proc` executable (camera, MSP, undistort, BEV) (Finished: ____-__-__ __:__)
- [ ] MP2-T1.a Publish BEV descriptors into `Q_BEV_TO_TRACK` and latest snapshot (Finished: ____-__-__ __:__)
- [ ] MP2-T2 Create `track_vel_proc` executable (tracking, velocity, registration) (Finished: ____-__-__ __:__)
- [ ] MP2-T2.a Publish `TrackResultDesc` into `Q_TRACK_TO_MAP` (and optional web queue) (Finished: ____-__-__ __:__)
- [ ] MP2-T3 Create `map_proc` executable (minimap update, drift/confidence) (Finished: ____-__-__ __:__)
- [ ] MP2-T3.a Publish `MapFrameDesc` into `Q_MAP_TO_WEB` and health snapshot (Finished: ____-__-__ __:__)
- [ ] MP2-T4 Create `web_ctrl_proc` executable (MJPEG serving, health/control endpoints) (Finished: ____-__-__ __:__)
- [ ] MP2-T4.a Consume latest BEV/map descriptors and serve stable streams (Finished: ____-__-__ __:__)

### Milestone MP3: Control Plane and Supervision
- [ ] MP3-T1 Implement Unix-domain control protocol (`ReloadConfig`, pause/resume map, health request) (Finished: ____-__-__ __:__)
- [ ] MP3-T2 Implement process supervisor (systemd units or launcher script with restart policy) (Finished: ____-__-__ __:__)
- [ ] MP3-T2.a Add startup ordering and readiness checks for SHM/rings (Finished: ____-__-__ __:__)
- [ ] MP3-T3 Implement graceful shutdown and reattach-on-restart behavior (Finished: ____-__-__ __:__)
- [ ] MP3-T3.a Reset/rehydrate logic on protocol mismatch or stale SHM generation (Finished: ____-__-__ __:__)

### Milestone MP4: CPU Scheduling and Memory Determinism
- [ ] MP4-T1 Implement CPU affinity policy for 4-process topology (Finished: ____-__-__ __:__)
- [ ] MP4-T2 Implement RT scheduling policy (`SCHED_FIFO`) with fallback path (Finished: ____-__-__ __:__)
- [ ] MP4-T2.a Add capability/privilege checks + warnings when RT unavailable (Finished: ____-__-__ __:__)
- [ ] MP4-T3 Implement `mlockall` and startup pre-allocation of hot-path buffers (Finished: ____-__-__ __:__)
- [ ] MP4-T4 Remove/guard dynamic allocations from per-frame hot loops (Finished: ____-__-__ __:__)

### Milestone MP5: Validation, Benchmarks, and Hardening
- [ ] MP5-T1 Add IPC correctness tests (ring wraparound, generation mismatch, refcount lifecycle) (Finished: ____-__-__ __:__)
- [ ] MP5-T2 Add overload tests to verify latency-bounded drop-old behavior (Finished: ____-__-__ __:__)
- [ ] MP5-T3 Add crash/restart resilience tests per process (Finished: ____-__-__ __:__)
- [ ] MP5-T4 Add benchmark suite for stage latency + end-to-end P50/P95/P99 + FPS (Finished: ____-__-__ __:__)
- [ ] MP5-T4.a Compare against serial baseline and generate regression report artifact (Finished: ____-__-__ __:__)
- [ ] MP5-T5 Add long-run soak test (30+ min) for queue stability and leak detection (Finished: ____-__-__ __:__)
- [ ] MP5-T6 Final tuning pass to prepare >15 FPS target envelope (Finished: ____-__-__ __:__)

## Priority Order (Foundation First)
1. MP0 (`IPC Foundations`)
2. MP1 (`Single-Binary IPC Boundaries`)
3. MP2 (`Process Split`)
4. MP3 (`Control + Supervision`)
5. MP4 (`Scheduling + Determinism`)
6. MP5 (`Validation + Tuning`)

Dependency logic:
1. MP1 depends on MP0.
2. MP2 depends on MP1.
3. MP3 depends on MP2.
4. MP4 depends on MP2 (and partially MP3 for startup order).
5. MP5 depends on MP2/MP3/MP4.
