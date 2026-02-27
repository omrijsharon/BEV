#pragma once

#include <string>

namespace bev {

struct TrackingConfig {
    int num_keypoints{80};
    float processing_scale{1.0F}; // (0,1], downscale only for tracking stage
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

struct IpcConfig {
    int frame_pool_slots{32};
    int frame_payload_bytes{921600}; // 640*480*3 default
    int frame_pool_slots_camera{16};
    int frame_pool_slots_bev{32};
    int frame_pool_slots_map{16};
    int frame_payload_bytes_camera{921600};
    int frame_payload_bytes_bev{307200}; // 640*480*1
    int frame_payload_bytes_map{4000000}; // default 2000*2000*1
    int ring_capacity_cam_to_bev{256};
    int ring_capacity_fc_to_bev{512};
    int ring_capacity_bev_to_track{256};
    int ring_capacity_track_to_map{256};
    int ring_capacity_track_to_web{128};
    int ring_capacity_map_to_web{128};
    int max_attitude_age_ms{100};
    int max_interp_gap_ms{60};
    int startup_min_fc_buffer{4};
    int web_jpeg_quality_default{75};
    int web_jpeg_quality_min{45};
    int web_quality_step{5};
    int web_drop_threshold_per_sec{50};
    int web_dashboard_period_ms{5};
    std::string web_body_view_mode{"rotate_world_client"}; // rotate_world_client | rp_only_backend
    bool adaptive_camera_fps_enable{true};
    int adaptive_camera_fps_min{8};
    int adaptive_camera_fps_max{30};
    int adaptive_camera_fps_step{1};
    int adaptive_backlog_low{1};
    int adaptive_backlog_high{8};
    int adaptive_eval_period_ms{1000};
    int adaptive_stable_intervals_for_up{2};
    int adaptive_stable_intervals_for_down{1};
    bool thermal_mitigation_enable{true};
    double thermal_limit_c{80.0};
    double thermal_resume_c{75.0};
    int thermal_fps_cap{12};
    int thermal_check_period_ms{1000};
    std::string shm_namespace{"/bev_ipc"};
    std::string queue_policy{"drop_old_keep_latest"};
    bool use_eventfd{true};
};

struct AppConfig {
    CameraConfig camera;
    CameraMountConfig camera_mount;
    TrackingConfig tracking;
    MotionConfig motion;
    MapConfig map;
    IpcConfig ipc;
};

bool loadConfig(const std::string& path, AppConfig& out, std::string& error);
bool validateConfig(const AppConfig& cfg, std::string& error);

}  // namespace bev
