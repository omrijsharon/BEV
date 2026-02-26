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
            << "  expand_margin_px: 100\n";
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
    std::remove(smoke_cfg.c_str());

    return 0;
}
