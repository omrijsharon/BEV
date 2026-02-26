# Camera Mount Orientation Verification Runbook

## Goal
Verify camera-to-IMU orientation settings (`camera_mount`) and yaw sign (`negate_yaw`) using short controlled maneuvers.

## Config Fields
- `camera_mount.roll_deg`
- `camera_mount.pitch_deg`
- `camera_mount.yaw_deg`
- `camera_mount.negate_yaw`
- `camera_mount.enable_roll_sign_ab_test`
- `camera_mount.ab_test_window_frames`

## Pre-Flight Setup
1. Place drone on bench, props removed.
2. Start app and open:
   - `http://localhost:8080/stream/main`
   - `http://localhost:8080/stream/map`
3. Keep `enable_roll_sign_ab_test: true` initially.

## Bench Verification Steps
1. Yaw-only test (slow CW then CCW)
- Expected: BEV should rotate consistently opposite real camera view motion after compensation.
- If direction is wrong, toggle `negate_yaw`.

2. Pitch-only test (nose up/down)
- Expected: horizon/ground texture stabilization should improve, not worsen.
- If compensation diverges, check pitch sign and mount pitch value.

3. Roll-only test (right wing down/up)
- Expected: lateral stabilization should improve.
- If image appears 90 deg swapped, adjust `roll_deg` sign/magnitude.

4. Sensor-plane 90 deg ambiguity check
- Keep `roll_deg` magnitude, compare `+abs(roll_deg)` vs `-abs(roll_deg)`.
- Use log line:
  - `[Mount-AB] ... recommended_roll_sign=...`
- Set final `roll_deg` sign to recommended one.

## Flight Verification (Short Hover)
1. Hover over textured ground.
2. Check frame-to-frame stability in `/stream/main`.
3. Confirm `/stream/map` grows smoothly and does not pause often.
4. If map pauses frequently, inspect confidence and tune mount parameters before tracker tuning.

## Acceptance
- BEV jitter noticeably reduced during yaw/pitch/roll motion.
- A/B roll sign converges to stable recommendation.
- Map overlay confidence remains high during steady motion.