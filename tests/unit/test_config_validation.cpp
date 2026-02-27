#include "core/config.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

int main() {
    bev::AppConfig cfg;
    std::string error;

    if (!bev::validateConfig(cfg, error)) {
        std::cerr << "default config should validate\n";
        return 1;
    }

    cfg.tracking.template_size = 20;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "even template_size should fail validation\n";
        return 1;
    }

    cfg.tracking.template_size = 21;
    cfg.tracking.processing_scale = 1.5F;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "processing_scale > 1 should fail validation\n";
        return 1;
    }
    cfg.tracking.processing_scale = 1.0F;
    cfg.motion.alpha = 1.5F;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "alpha > 1 should fail validation\n";
        return 1;
    }

    cfg.motion.alpha = 0.5F;
    cfg.camera_mount.roll_deg = 181.0;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "camera_mount roll out of range should fail validation\n";
        return 1;
    }
    cfg.camera_mount.roll_deg = 90.0;
    cfg.ipc.ring_capacity_bev_to_track = 63;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "non-power-of-two ring capacity should fail validation\n";
        return 1;
    }
    cfg.ipc.ring_capacity_bev_to_track = 64;
    cfg.ipc.web_jpeg_quality_default = 40;
    cfg.ipc.web_jpeg_quality_min = 50;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "web jpeg quality min > default should fail validation\n";
        return 1;
    }
    cfg.ipc.web_jpeg_quality_default = 75;
    cfg.ipc.web_jpeg_quality_min = 45;
    cfg.ipc.queue_policy = "unsupported_policy";
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "unsupported ipc queue policy should fail validation\n";
        return 1;
    }
    cfg.ipc.queue_policy = "drop_old_keep_latest";
    cfg.ipc.web_body_view_mode = "invalid_mode";
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "unsupported web body mode should fail validation\n";
        return 1;
    }
    cfg.ipc.web_body_view_mode = "rotate_world_client";
    cfg.ipc.adaptive_camera_fps_min = 20;
    cfg.ipc.adaptive_camera_fps_max = 10;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "adaptive fps min > max should fail validation\n";
        return 1;
    }
    cfg.ipc.adaptive_camera_fps_min = 8;
    cfg.ipc.adaptive_camera_fps_max = 30;
    cfg.ipc.thermal_resume_c = 85.0;
    cfg.ipc.thermal_limit_c = 80.0;
    if (bev::validateConfig(cfg, error)) {
        std::cerr << "thermal resume > limit should fail validation\n";
        return 1;
    }
    cfg.ipc.thermal_resume_c = 75.0;
    cfg.ipc.thermal_limit_c = 80.0;

    // Loader robustness: ensure config with gstreamer pipeline containing '!' parses.
    const std::string smoke_cfg = "build_test_cfg_gst.yaml";
    {
        std::ofstream ofs(smoke_cfg);
        ofs << "camera:\n"
            << "  width: 640\n"
            << "  height: 480\n"
            << "  fps: 15\n"
            << "  source_mode: \"gstreamer\"\n"
            << "  device_index: 0\n"
            << "  gstreamer_pipeline: \"videotestsrc is-live=true pattern=ball ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1 sync=false\"\n"
            << "camera_mount:\n"
            << "  roll_deg: 90.0\n"
            << "  pitch_deg: 90.0\n"
            << "  yaw_deg: 0.0\n"
            << "  negate_yaw: true\n"
            << "  enable_roll_sign_ab_test: false\n"
            << "  ab_test_window_frames: 60\n"
            << "tracking:\n"
            << "  num_keypoints: 80\n"
            << "  processing_scale: 0.5\n"
            << "  template_size: 21\n"
            << "  coarse_crop_size: 64\n"
            << "  fine_crop_size: 24\n"
            << "  max_distance_from_keypoint_center: 30\n"
            << "  min_valid_tracks: 20\n"
            << "  ransac_reproj_threshold_px: 3.0\n"
            << "motion:\n"
            << "  alpha: 0.5\n"
            << "  threshold: 30\n"
            << "map:\n"
            << "  initial_width: 800\n"
            << "  initial_height: 600\n"
            << "  expand_margin_px: 100\n"
            << "ipc:\n"
            << "  frame_pool_slots: 16\n"
            << "  frame_payload_bytes: 307200\n"
            << "  ring_capacity_bev_to_track: 64\n"
            << "  ring_capacity_track_to_map: 64\n"
            << "  ring_capacity_track_to_web: 64\n"
            << "  ring_capacity_map_to_web: 64\n"
            << "  web_body_view_mode: \"rp_only_backend\"\n"
            << "  adaptive_camera_fps_enable: true\n"
            << "  adaptive_camera_fps_min: 8\n"
            << "  adaptive_camera_fps_max: 24\n"
            << "  adaptive_camera_fps_step: 2\n"
            << "  adaptive_backlog_low: 1\n"
            << "  adaptive_backlog_high: 6\n"
            << "  adaptive_eval_period_ms: 1000\n"
            << "  adaptive_stable_intervals_for_up: 2\n"
            << "  adaptive_stable_intervals_for_down: 1\n"
            << "  thermal_mitigation_enable: true\n"
            << "  thermal_limit_c: 80.0\n"
            << "  thermal_resume_c: 75.0\n"
            << "  thermal_fps_cap: 12\n"
            << "  thermal_check_period_ms: 1000\n"
            << "  queue_policy: \"drop_old_keep_latest\"\n"
            << "  use_eventfd: true\n";
    }

    bev::AppConfig loaded;
    if (!bev::loadConfig(smoke_cfg, loaded, error)) {
        std::cerr << "loadConfig should parse gstreamer pipeline with exclamation marks: " << error << "\n";
        return 1;
    }
    if (loaded.camera.source_mode != "gstreamer") {
        std::cerr << "loaded source mode mismatch\n";
        return 1;
    }
    if (loaded.ipc.frame_pool_slots != 16 || loaded.ipc.queue_policy != "drop_old_keep_latest") {
        std::cerr << "loaded ipc config mismatch\n";
        return 1;
    }
    if (loaded.ipc.web_body_view_mode != "rp_only_backend") {
        std::cerr << "loaded web body mode mismatch\n";
        return 1;
    }
    if (loaded.tracking.processing_scale != 0.5F) {
        std::cerr << "loaded tracking processing_scale mismatch\n";
        return 1;
    }
    if (!loaded.ipc.adaptive_camera_fps_enable || loaded.ipc.adaptive_camera_fps_max != 24) {
        std::cerr << "loaded adaptive fps config mismatch\n";
        return 1;
    }
    if (!loaded.ipc.thermal_mitigation_enable || loaded.ipc.thermal_fps_cap != 12) {
        std::cerr << "loaded thermal config mismatch\n";
        return 1;
    }
    std::remove(smoke_cfg.c_str());

    return 0;
}
