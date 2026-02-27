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
#include "ipc/descriptors.hpp"
#include "ipc/frame_pool.hpp"
#include "ipc/spsc_ring.hpp"
#include "web/dashboard_renderer.hpp"
#include "web/mjpeg_server.hpp"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#ifdef __linux__
#include <unistd.h>
#endif

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

    bev::ipc::SpscRing<bev::ipc::BevFrameDesc> q_bev_to_track(
        static_cast<std::size_t>(config.ipc.ring_capacity_bev_to_track));
    if (!q_bev_to_track.valid()) {
        std::cerr << "Invalid ipc.ring_capacity_bev_to_track, expected power-of-two > 1\n";
        return 1;
    }

    bev::ipc::ShmFramePool frame_pool;
    bool ipc_singleproc_enabled = false;
    std::string frame_pool_error;
#ifdef __linux__
    const std::string shm_name = "/bev_frame_pool_" + std::to_string(static_cast<long long>(::getpid()));
    if (!frame_pool.create(
            shm_name,
            static_cast<std::size_t>(config.ipc.frame_pool_slots),
            static_cast<std::size_t>(config.ipc.frame_payload_bytes),
            frame_pool_error)) {
        std::cerr << "IPC frame pool init failed, running fallback direct handoff: " << frame_pool_error << '\n';
    } else {
        ipc_singleproc_enabled = true;
    }
#else
    std::cout << "IPC SHM frame pool unsupported on this OS, running fallback direct handoff\n";
#endif

    std::cout << "Streaming on http://localhost:8080/stream/main\n";
    std::cout << "Mini-map on  http://localhost:8080/stream/map\n";
    std::cout << "Camera source: " << camera.activeSourceDescription() << '\n';
    std::cout << "Press Ctrl+C to stop.\n";

    bev::Telemetry telemetry;
    int frame_counter = 0;
    int64_t window_start_ns = bev::nowSteadyNs();
    uint64_t sequence_id = 0;

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
        cv::Mat tracking_gray = bev_gray;

        bool producer_slot_held = false;
        bool consumer_slot_held = false;
        std::size_t producer_slot_idx = 0;
        std::size_t consumer_slot_idx = 0;

        if (ipc_singleproc_enabled && frame_pool.acquireWritableSlot(producer_slot_idx)) {
            producer_slot_held = true;
            const std::size_t required_bytes = bev_gray.total() * bev_gray.elemSize();
            const std::size_t payload_bytes = frame_pool.payloadBytesPerSlot();
            uint8_t* dst = frame_pool.payloadPtr(producer_slot_idx);

            if (dst != nullptr && required_bytes <= payload_bytes) {
                if (bev_gray.isContinuous()) {
                    std::memcpy(dst, bev_gray.data, required_bytes);
                } else {
                    for (int r = 0; r < bev_gray.rows; ++r) {
                        std::memcpy(
                            dst + static_cast<std::size_t>(r) * bev_gray.cols * bev_gray.elemSize(),
                            bev_gray.ptr(r),
                            static_cast<std::size_t>(bev_gray.cols) * bev_gray.elemSize());
                    }
                }

                uint32_t generation = 0;
                if (frame_pool.publishWrittenSlot(
                        producer_slot_idx,
                        static_cast<uint32_t>(bev_gray.cols),
                        static_cast<uint32_t>(bev_gray.rows),
                        static_cast<uint32_t>(bev_gray.type()),
                        static_cast<uint32_t>(bev_gray.cols * bev_gray.elemSize()),
                        packet.timestamp_ns,
                        generation)) {
                    auto desc = bev::ipc::makeBevFrameDesc();
                    desc.frame_slot_id = static_cast<uint32_t>(producer_slot_idx);
                    desc.generation = generation;
                    desc.width = static_cast<uint32_t>(bev_gray.cols);
                    desc.height = static_cast<uint32_t>(bev_gray.rows);
                    desc.type = static_cast<uint32_t>(bev_gray.type());
                    desc.stride = static_cast<uint32_t>(bev_gray.cols * bev_gray.elemSize());
                    desc.sequence_id = sequence_id++;
                    desc.timestamp_ns = packet.timestamp_ns;
                    (void)q_bev_to_track.pushDropOldest(desc);

                    bev::ipc::BevFrameDesc latest_desc{};
                    bool got_desc = false;
                    while (q_bev_to_track.pop(latest_desc)) {
                        got_desc = true;
                    }
                    if (got_desc) {
                        consumer_slot_idx = static_cast<std::size_t>(latest_desc.frame_slot_id);
                        if (frame_pool.acquireReadSlot(consumer_slot_idx, latest_desc.generation)) {
                            consumer_slot_held = true;
                            const auto* hdr = frame_pool.header(consumer_slot_idx);
                            uint8_t* src = frame_pool.payloadPtr(consumer_slot_idx);
                            if (hdr != nullptr && src != nullptr) {
                                tracking_gray = cv::Mat(
                                    static_cast<int>(hdr->height),
                                    static_cast<int>(hdr->width),
                                    static_cast<int>(hdr->type),
                                    src,
                                    static_cast<std::size_t>(hdr->stride));
                            }
                        }
                    }
                }
            }
        }

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
            tracking_gray,
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
            const auto grid = bev::KeypointInitializer::initializeGrid(tracking_gray.size(), rows, cols, 20);
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
            minimap.addFrame(tracking_gray, H_identity);
        }

        if (consumer_slot_held) {
            frame_pool.releaseSlot(consumer_slot_idx);
        }
        if (producer_slot_held) {
            frame_pool.releaseSlot(producer_slot_idx);
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
