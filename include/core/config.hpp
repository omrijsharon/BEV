#pragma once

#include <string>

namespace bev {

struct TrackingConfig {
    int num_keypoints{80};
    int template_size{21};
    int coarse_crop_size{64};
    int fine_crop_size{24};
    float max_distance_from_keypoint_center{30.0F};
    int min_valid_tracks{20};
    double ransac_reproj_threshold_px{3.0};
};

struct MotionConfig {
    float alpha{0.5F};
    int threshold{30};
};

struct CameraConfig {
    int width{640};
    int height{480};
    int fps{15};
    std::string source_mode{"v4l2"};  // v4l2 | gstreamer
    int device_index{0};
    std::string gstreamer_pipeline{
        "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! appsink drop=true max-buffers=1"};
};

struct CameraMountConfig {
    double roll_deg{90.0};
    double pitch_deg{90.0};
    double yaw_deg{0.0};
    bool negate_yaw{true};
    bool enable_roll_sign_ab_test{true};
    int ab_test_window_frames{120};
};

struct MapConfig {
    int initial_width{2000};
    int initial_height{2000};
    int expand_margin_px{150};
};

struct AppConfig {
    CameraConfig camera;
    CameraMountConfig camera_mount;
    TrackingConfig tracking;
    MotionConfig motion;
    MapConfig map;
};

bool loadConfig(const std::string& path, AppConfig& out, std::string& error);
bool validateConfig(const AppConfig& cfg, std::string& error);

}  // namespace bev
