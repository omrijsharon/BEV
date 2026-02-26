#include "bev/bev_warp.hpp"
#include "camera/camera_calibration.hpp"
#include "camera/camera_ingest.hpp"
#include "core/config.hpp"
#include "core/math_utils.hpp"
#include "core/telemetry.hpp"
#include "core/time_utils.hpp"
#include "map/minimap_builder.hpp"
#include "tracking/keypoint_initializer.hpp"
#include "tracking/track_manager.hpp"
#include "web/dashboard_renderer.hpp"
#include "web/mjpeg_server.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void onSigInt(int) {
    g_running.store(false);
}

struct RollSignABState {
    int window_count{0};
    double score_pos{0.0};
    double score_neg{0.0};
    cv::Mat prev_pos_gray;
    cv::Mat prev_neg_gray;
};

double frameDeltaScore(const cv::Mat& prev_gray, const cv::Mat& curr_gray) {
    if (prev_gray.empty() || curr_gray.empty() || prev_gray.size() != curr_gray.size()) {
        return 0.0;
    }
    cv::Mat diff;
    cv::absdiff(prev_gray, curr_gray, diff);
    return cv::mean(diff)[0];
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, onSigInt);

    const std::string config_path = (argc > 1) ? argv[1] : "config/config.yaml";
    const std::string calib_path = (argc > 2) ? argv[2] : "config/camera_calibration.yaml";

    bev::AppConfig config;
    std::string error;
    if (!bev::loadConfig(config_path, config, error)) {
        std::cerr << "Config load failed: " << error << '\n';
        return 1;
    }

    bev::CameraCalibration calibration;
    if (!calibration.loadFromFile(calib_path, error)) {
        std::cerr << "Calibration load failed: " << error << '\n';
        return 1;
    }

    bev::CameraIngest camera;
    camera.setCalibration(calibration.data());
    if (!camera.openDevice(config.camera, error)) {
        std::cerr << "Camera open failed: " << error << '\n';
        return 1;
    }

    bev::BEVWarp bev_warp(calibration.data().K);
    bev::MJPEGServer server;
    if (!server.start(8080)) {
        std::cerr << "MJPEG server failed to start on port 8080\n";
        return 1;
    }
    server.addRoute("/stream/main");
    server.addRoute("/stream/map");

    bev::MinimapBuilder minimap;
    if (!minimap.initialize(config.map.initial_width, config.map.initial_height)) {
        std::cerr << "Failed to initialize minimap canvas\n";
        return 1;
    }

    bev::TrackManager track_manager(config.tracking.max_distance_from_keypoint_center);
    bev::TrackManager::RecoveryState recovery{};
    std::vector<bev::TrackPoint> tracks;
    RollSignABState ab_state{};

    std::cout << "Streaming on http://localhost:8080/stream/main\n";
    std::cout << "Mini-map on  http://localhost:8080/stream/map\n";
    std::cout << "Camera source: " << camera.activeSourceDescription() << '\n';
    std::cout << "Press Ctrl+C to stop.\n";

    bev::Telemetry telemetry;
    int frame_counter = 0;
    int64_t window_start_ns = bev::nowSteadyNs();

    while (g_running.load()) {
        bev::FramePacket packet;
        if (!camera.captureFrame(packet, error)) {
            std::cerr << "Capture failed: " << error << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        bev::AttitudeSample sample{};
        sample.timestamp_ns = packet.timestamp_ns;
        sample.roll_rad = 0.0;
        sample.pitch_rad = 0.0;
        sample.yaw_rad = 0.0;

        const cv::Matx33d R_cam_world = bev::composeCameraWorldRotation(
            sample.roll_rad,
            sample.pitch_rad,
            sample.yaw_rad,
            config.camera_mount.negate_yaw,
            config.camera_mount.roll_deg,
            config.camera_mount.pitch_deg,
            config.camera_mount.yaw_deg);

        cv::Mat bev_frame;
        bev_warp.warpToBEV(packet.undistorted_bgr, bev_frame, R_cam_world);
        cv::Mat bev_gray;
        cv::cvtColor(bev_frame, bev_gray, cv::COLOR_BGR2GRAY);

        if (config.camera_mount.enable_roll_sign_ab_test) {
            const double roll_abs = std::abs(config.camera_mount.roll_deg);
            const cv::Matx33d R_pos = bev::composeCameraWorldRotation(
                sample.roll_rad, sample.pitch_rad, sample.yaw_rad,
                config.camera_mount.negate_yaw,
                roll_abs, config.camera_mount.pitch_deg, config.camera_mount.yaw_deg);
            const cv::Matx33d R_neg = bev::composeCameraWorldRotation(
                sample.roll_rad, sample.pitch_rad, sample.yaw_rad,
                config.camera_mount.negate_yaw,
                -roll_abs, config.camera_mount.pitch_deg, config.camera_mount.yaw_deg);

            cv::Mat bev_pos;
            cv::Mat bev_neg;
            cv::Mat gray_pos;
            cv::Mat gray_neg;
            bev_warp.warpToBEV(packet.undistorted_bgr, bev_pos, R_pos);
            bev_warp.warpToBEV(packet.undistorted_bgr, bev_neg, R_neg);
            cv::cvtColor(bev_pos, gray_pos, cv::COLOR_BGR2GRAY);
            cv::cvtColor(bev_neg, gray_neg, cv::COLOR_BGR2GRAY);

            if (!ab_state.prev_pos_gray.empty() && !ab_state.prev_neg_gray.empty()) {
                ab_state.score_pos += frameDeltaScore(ab_state.prev_pos_gray, gray_pos);
                ab_state.score_neg += frameDeltaScore(ab_state.prev_neg_gray, gray_neg);
                ab_state.window_count += 1;
            }
            ab_state.prev_pos_gray = gray_pos;
            ab_state.prev_neg_gray = gray_neg;

            if (ab_state.window_count >= config.camera_mount.ab_test_window_frames) {
                const char* recommend = (ab_state.score_pos <= ab_state.score_neg) ? "+abs(roll_deg)" : "-abs(roll_deg)";
                std::cout << "[Mount-AB] window=" << ab_state.window_count
                          << " score_pos=" << ab_state.score_pos
                          << " score_neg=" << ab_state.score_neg
                          << " recommended_roll_sign=" << recommend << '\n';
                ab_state.window_count = 0;
                ab_state.score_pos = 0.0;
                ab_state.score_neg = 0.0;
            }
        }

        frame_counter++;
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - window_start_ns >= 1000000000LL) {
            telemetry.setFps(frame_counter);
            frame_counter = 0;
            window_start_ns = now_ns;
        }

        const auto points = bev::KeypointInitializer::initializeShiTomasi(
            bev_gray,
            config.tracking.num_keypoints,
            0.01,
            8.0);

        tracks.clear();
        tracks.reserve(points.size());
        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            bev::TrackPoint tp;
            tp.id = i;
            tp.pos = points[i];
            tp.vel_px_s = cv::Point2f(0.0F, 0.0F);
            tp.confidence = 1.0F;
            tp.valid = true;
            tracks.push_back(tp);
        }

        recovery = bev::TrackManager::updateRecoveryState(
            recovery,
            tracks,
            config.tracking.min_valid_tracks,
            0.5F,
            5);

        if (recovery.should_reinitialize) {
            // Reinitialize to deterministic grid when texture confidence collapses.
            const int rows = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(config.tracking.num_keypoints))));
            const int cols = std::max(1, config.tracking.num_keypoints / rows);
            const auto grid = bev::KeypointInitializer::initializeGrid(bev_gray.size(), rows, cols, 20);
            tracks.clear();
            tracks.reserve(grid.size());
            for (int i = 0; i < static_cast<int>(grid.size()); ++i) {
                bev::TrackPoint tp;
                tp.id = i;
                tp.pos = grid[i];
                tp.confidence = 0.7F;
                tp.valid = true;
                tracks.push_back(tp);
            }
        }

        if (!recovery.map_updates_paused) {
            const cv::Mat H_identity = cv::Mat::eye(3, 3, CV_64F);
            minimap.addFrame(bev_gray, H_identity);
        }

        bev::DashboardStatus status{};
        status.fps = telemetry.snapshot().fps;
        status.valid_tracks = bev::TrackManager::countValidTracks(tracks);
        status.inliers = 0;
        status.msp_fresh = false;
        bev::DashboardRenderer::renderStatusOverlay(bev_frame, status);

        cv::Mat map_bgr;
        cv::cvtColor(minimap.canvas(), map_bgr, cv::COLOR_GRAY2BGR);
        const auto map_stats = minimap.stats();
        const cv::Mat T = minimap.mapOffsetTransform();
        const double drift_px = std::sqrt(
            T.at<double>(0, 2) * T.at<double>(0, 2) +
            T.at<double>(1, 2) * T.at<double>(1, 2));

        bev::MapDashboardStatus map_status{};
        map_status.drift_px = drift_px;
        map_status.confidence = std::max(0.0, std::min(1.0, static_cast<double>(bev::TrackManager::meanConfidence(tracks))));
        map_status.coverage_ratio = map_stats.coverage_ratio;
        map_status.map_paused = recovery.map_updates_paused;
        bev::DashboardRenderer::renderMapOverlay(map_bgr, map_status);

        server.updateFrame("/stream/main", bev_frame, 80);
        server.updateFrame("/stream/map", map_bgr, 80);
    }

    server.stop();
    camera.close();
    return 0;
}
