#include "bev/bev_warp.hpp"
#include "camera/camera_calibration.hpp"
#include "camera/camera_ingest.hpp"
#include "core/config.hpp"
#include "core/fps_governor.hpp"
#include "core/math_utils.hpp"
#include "core/time_utils.hpp"
#include "ipc/descriptors.hpp"
#include "ipc/control_plane.hpp"
#include "ipc/frame_pool.hpp"
#include "ipc/pipeline_names.hpp"
#include "ipc/shm_spsc_ring.hpp"
#include "ipc/shared_state.hpp"
#include "map/minimap_builder.hpp"
#include "msp/attitude_sync.hpp"
#include "tracking/keypoint_initializer.hpp"
#include "tracking/template_tracker.hpp"
#include "tracking/track_manager.hpp"
#include "web/mjpeg_server.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#ifdef __linux__
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fstream>
#endif

namespace {

std::atomic<bool> g_running{true};

enum class WebBodyViewMode {
    RotateWorldClient = 0,
    RpOnlyBackend = 1,
};

WebBodyViewMode parseWebBodyViewMode(const std::string& s) {
    if (s == "rp_only_backend") {
        return WebBodyViewMode::RpOnlyBackend;
    }
    return WebBodyViewMode::RotateWorldClient;
}

uint64_t packYawRad(float yaw_rad) {
    uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(yaw_rad), "float/u32 size mismatch");
    std::memcpy(&bits, &yaw_rad, sizeof(bits));
    return static_cast<uint64_t>(bits);
}

float unpackYawRad(uint64_t packed) {
    uint32_t bits = static_cast<uint32_t>(packed & 0xFFFFFFFFULL);
    float yaw = 0.0F;
    std::memcpy(&yaw, &bits, sizeof(yaw));
    return yaw;
}

bool readSocTempMilliC(uint32_t& out_milli_c) {
#ifdef __linux__
    std::ifstream ifs("/sys/class/thermal/thermal_zone0/temp");
    if (!ifs.is_open()) {
        return false;
    }
    long long v = 0;
    ifs >> v;
    if (!ifs.good()) {
        return false;
    }
    if (v < 0) {
        return false;
    }
    out_milli_c = static_cast<uint32_t>(v);
    return true;
#else
    (void)out_milli_c;
    return false;
#endif
}

void onSigInt(int) {
    g_running.store(false);
}

bool copyMatToPayload(const cv::Mat& m, uint8_t* dst, std::size_t payload_bytes) {
    if (m.empty() || dst == nullptr) {
        return false;
    }
    const std::size_t required = m.total() * m.elemSize();
    if (required > payload_bytes) {
        return false;
    }
    if (m.isContinuous()) {
        std::memcpy(dst, m.data, required);
        return true;
    }
    const std::size_t row_bytes = static_cast<std::size_t>(m.cols) * m.elemSize();
    for (int r = 0; r < m.rows; ++r) {
        std::memcpy(dst + static_cast<std::size_t>(r) * row_bytes, m.ptr(r), row_bytes);
    }
    return true;
}

cv::Mat matFromSlot(bev::ipc::ShmFramePool& pool, std::size_t slot) {
    const auto* hdr = pool.header(slot);
    auto* ptr = pool.payloadPtr(slot);
    if (hdr == nullptr || ptr == nullptr) {
        return {};
    }
    return cv::Mat(
        static_cast<int>(hdr->height),
        static_cast<int>(hdr->width),
        static_cast<int>(hdr->type),
        ptr,
        static_cast<std::size_t>(hdr->stride));
}

bool attachSharedStateWithRetry(bev::ipc::SharedState& state, const std::string& name, int tries, int delay_ms, std::string& err) {
    for (int i = 0; i < tries; ++i) {
        if (state.attach(name, err)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    return false;
}

bool maybeReloadConfig(
    bev::AppConfig& cfg_runtime,
    const std::string& config_path,
    bev::ipc::SharedState& shared,
    uint32_t& last_reload_generation,
    const char* role_name) {
    const uint32_t g = shared.reloadGeneration();
    if (g == last_reload_generation) {
        return false;
    }
    bev::AppConfig new_cfg = cfg_runtime;
    std::string err;
    if (!bev::loadConfig(config_path, new_cfg, err)) {
        std::cerr << role_name << ": reload config failed: " << err << "\n";
        last_reload_generation = g;
        return false;
    }
    cfg_runtime = new_cfg;
    last_reload_generation = g;
    std::cerr << role_name << ": config reloaded (generation " << g << ")\n";
    return true;
}

template <typename Fn>
bool retryResourceInit(Fn&& fn, int tries, int delay_ms, bev::ipc::SharedState* shared, std::string& err) {
    for (int i = 0; i < tries; ++i) {
        if (fn(err)) {
            if (shared != nullptr) {
                shared->setProtocolMismatch(false);
            }
            return true;
        }
        if (shared != nullptr && err.find("mismatch") != std::string::npos) {
            shared->setProtocolMismatch(true);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    return false;
}

void applyRoleScheduling(const std::string& role) {
#ifdef __linux__
    int core = -1;
    int priority = 0;
    if (role == "camera") {
        core = 0;
        priority = 70;
    } else if (role == "fc") {
        core = 0;
        priority = 60;
    } else if (role == "bev") {
        core = 1;
        priority = 65;
    } else if (role == "track") {
        core = 2;
        priority = 55;
    } else if (role == "map") {
        core = 3;
        priority = 45;
    } else if (role == "web") {
        core = 3;
        priority = 0;
    }

    if (core >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(core, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) {
            std::cerr << "warning: sched_setaffinity failed for role " << role << "\n";
        }
    }

    if (priority > 0) {
        sched_param param{};
        param.sched_priority = priority;
        if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
            std::cerr << "warning: SCHED_FIFO unavailable for role " << role << ", continuing without RT\n";
        }
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "warning: mlockall unavailable for role " << role << "\n";
    }
#else
    (void)role;
#endif
}

int runCamera(const bev::AppConfig& cfg, const std::string& config_path, const std::string& calib_path) {
    bev::AppConfig cfg_runtime = cfg;
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    bev::ipc::SharedState shared;
    std::string err;
    if (!shared.create(names.shared_state, err)) {
        std::cerr << "camera: create shared state failed: " << err << "\n";
        return 1;
    }
    shared.resetControlFlagsForStartup();
    shared.resetRoleHealthForStartup();
    bev::CameraCalibration calibration;
    bool has_calibration = calibration.loadFromFile(calib_path, err);
    if (!has_calibration) {
        std::cerr << "camera: calibration load failed, continuing without calibration: " << err << "\n";
    }

    bev::CameraIngest camera;
    if (has_calibration) {
        camera.setCalibration(calibration.data());
    }
    bool synthetic_mode = false;
    if (!camera.openDevice(cfg_runtime.camera, err)) {
        std::cerr << "camera: open failed, switching to synthetic source: " << err << "\n";
        synthetic_mode = true;
    }

    bev::ipc::ShmFramePool pool_cam;
    if (!pool_cam.create(
            names.pool_cam,
            static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_camera),
            static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_camera),
            err)) {
        std::cerr << "camera: create pool_cam failed: " << err << "\n";
        return 1;
    }
    bev::ipc::TypedShmSpscRing<bev::ipc::FrameDesc> q_cam;
    if (!q_cam.create(names.q_cam_to_bev, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_cam_to_bev), err)) {
        std::cerr << "camera: create q_cam_to_bev failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Camera, "camera");

    uint64_t seq = 0;
    uint32_t last_reload_generation = shared.reloadGeneration();
    int synth_tick = 0;
    int capture_fail_streak = 0;
    bev::AdaptiveFpsGovernor fps_governor(cfg_runtime.ipc);
    int target_fps = std::max(1, std::min(cfg_runtime.ipc.adaptive_camera_fps_max, cfg_runtime.camera.fps));
    int effective_target_fps = target_fps;
    int64_t last_eval_ns = bev::nowSteadyNs();
    uint64_t last_drop_count = 0;
    int64_t last_thermal_ns = bev::nowSteadyNs();
    bool thermal_throttled = false;
    uint32_t last_temp_milli_c = 0;
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        if (maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "camera")) {
            fps_governor.updateConfig(cfg_runtime.ipc);
            target_fps = std::max(1, std::min(cfg_runtime.ipc.adaptive_camera_fps_max, cfg_runtime.camera.fps));
            effective_target_fps = target_fps;
            thermal_throttled = false;
            if (!synthetic_mode) {
                camera.close();
                if (!camera.openDevice(cfg_runtime.camera, err)) {
                    std::cerr << "camera: reopen after reload failed, keeping synthetic mode: " << err << "\n";
                    synthetic_mode = true;
                }
            }
        }
        bev::FramePacket pkt;
        if (synthetic_mode) {
            const int w = std::max(32, cfg_runtime.camera.width);
            const int h = std::max(24, cfg_runtime.camera.height);
            pkt.timestamp_ns = bev::nowSteadyNs();
            pkt.raw_bgr = cv::Mat::zeros(h, w, CV_8UC3);
            const int cx = (synth_tick * 3) % w;
            const int cy = h / 2 + static_cast<int>(std::sin(static_cast<double>(synth_tick) * 0.05) * (h / 4));
            cv::circle(pkt.raw_bgr, cv::Point(cx, cy), 20, cv::Scalar(50, 200, 50), -1);
            cv::putText(pkt.raw_bgr, "synthetic", cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
            pkt.undistorted_bgr = pkt.raw_bgr;
            synth_tick++;
        } else if (!camera.captureFrame(pkt, err)) {
            capture_fail_streak++;
            if (capture_fail_streak > 30) {
                std::cerr << "camera: capture repeatedly failed, switching to synthetic source\n";
                synthetic_mode = true;
                camera.close();
                capture_fail_streak = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        } else {
            capture_fail_streak = 0;
        }
        std::size_t slot = 0;
        if (!pool_cam.acquireWritableSlot(slot)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        const bool copied = copyMatToPayload(pkt.undistorted_bgr, pool_cam.payloadPtr(slot), pool_cam.payloadBytesPerSlot());
        uint32_t gen = 0;
        bool published = false;
        if (copied && pool_cam.publishWrittenSlot(
                slot,
                static_cast<uint32_t>(pkt.undistorted_bgr.cols),
                static_cast<uint32_t>(pkt.undistorted_bgr.rows),
                static_cast<uint32_t>(pkt.undistorted_bgr.type()),
                static_cast<uint32_t>(pkt.undistorted_bgr.cols * pkt.undistorted_bgr.elemSize()),
                pkt.timestamp_ns,
                gen)) {
            auto d = bev::ipc::makeFrameDesc();
            d.frame_slot_id = static_cast<uint32_t>(slot);
            d.generation = gen;
            d.width = static_cast<uint32_t>(pkt.undistorted_bgr.cols);
            d.height = static_cast<uint32_t>(pkt.undistorted_bgr.rows);
            d.type = static_cast<uint32_t>(pkt.undistorted_bgr.type());
            d.stride = static_cast<uint32_t>(pkt.undistorted_bgr.cols * pkt.undistorted_bgr.elemSize());
            d.timestamp_ns = pkt.timestamp_ns;
            d.sequence_id = seq++;
            (void)q_cam.pushDropOldest(d);
            shared.updateHeartbeat(bev::ipc::RoleId::Camera, static_cast<uint64_t>(pkt.timestamp_ns), d.sequence_id, q_cam.dropCount());
            shared.publishLatestCamera(
                d.frame_slot_id,
                d.generation,
                d.width,
                d.height,
                d.type,
                d.stride,
                d.sequence_id,
                d.timestamp_ns);
            shared.recordStageKpi(
                bev::ipc::RoleId::Camera,
                static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
                q_cam.depth(),
                q_cam.dropCount());
            published = true;
        }
        if (!published) {
            pool_cam.releaseSlot(slot);
        }
        loop_count++;
        int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_eval_ns >= static_cast<int64_t>(cfg_runtime.ipc.adaptive_eval_period_ms) * 1000000LL) {
            uint64_t backlog = q_cam.depth();
            if (const auto* block = shared.block()) {
                backlog = std::max(
                    backlog,
                    block->stage_kpi[static_cast<uint32_t>(bev::ipc::RoleId::Bev)].queue_depth.load(std::memory_order_acquire));
                backlog = std::max(
                    backlog,
                    block->stage_kpi[static_cast<uint32_t>(bev::ipc::RoleId::Track)].queue_depth.load(std::memory_order_acquire));
            }
            const uint64_t drops = q_cam.dropCount();
            const uint64_t drop_delta = (drops >= last_drop_count) ? (drops - last_drop_count) : drops;
            last_drop_count = drops;
            target_fps = fps_governor.update(backlog, drop_delta);
            last_eval_ns = now_ns;
        }
        if (now_ns - last_thermal_ns >= static_cast<int64_t>(cfg_runtime.ipc.thermal_check_period_ms) * 1000000LL) {
            uint32_t temp_mc = 0U;
            if (readSocTempMilliC(temp_mc)) {
                last_temp_milli_c = temp_mc;
                const double temp_c = static_cast<double>(temp_mc) / 1000.0;
                if (cfg_runtime.ipc.thermal_mitigation_enable) {
                    if (thermal_throttled) {
                        if (temp_c <= cfg_runtime.ipc.thermal_resume_c) {
                            thermal_throttled = false;
                        }
                    } else if (temp_c >= cfg_runtime.ipc.thermal_limit_c) {
                        thermal_throttled = true;
                    }
                } else {
                    thermal_throttled = false;
                }
            }
            last_thermal_ns = now_ns;
        }
        effective_target_fps = target_fps;
        if (thermal_throttled) {
            effective_target_fps = std::min(effective_target_fps, std::max(1, cfg_runtime.ipc.thermal_fps_cap));
        }
        shared.publishThermal(last_temp_milli_c, thermal_throttled, static_cast<uint32_t>(effective_target_fps));
        const int64_t frame_period_ns = 1000000000LL / std::max(1, effective_target_fps);
        const int64_t elapsed_ns = now_ns - loop_start_ns;
        if (elapsed_ns < frame_period_ns) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(frame_period_ns - elapsed_ns));
            now_ns = bev::nowSteadyNs();
        }
        if (now_ns - last_log_ns >= 1000000000LL) {
            std::cerr << "[camera] loops=" << loop_count
                      << " q_cam_to_bev_depth=" << q_cam.depth()
                      << " q_cam_to_bev_dropped=" << q_cam.dropCount()
                      << " target_fps=" << target_fps
                      << " effective_fps=" << effective_target_fps
                      << " thermal_throttled=" << (thermal_throttled ? 1 : 0)
                      << " temp_c=" << (static_cast<double>(last_temp_milli_c) / 1000.0)
                      << " synthetic=" << (synthetic_mode ? 1 : 0)
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns;
        }
    }
    return 0;
}

int runFc(const bev::AppConfig& cfg, const std::string& config_path) {
    bev::AppConfig cfg_runtime = cfg;
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    std::string err;
    bev::ipc::SharedState shared;
    if (!attachSharedStateWithRetry(shared, names.shared_state, 100, 50, err)) {
        std::cerr << "fc: attach shared state failed: " << err << "\n";
        return 1;
    }
    bev::ipc::TypedShmSpscRing<bev::ipc::AttitudeDesc> q_fc;
    if (!q_fc.create(names.q_fc_to_bev, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_fc_to_bev), err)) {
        std::cerr << "fc: create q_fc_to_bev failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Fc, "fc");

    uint32_t sid = 0;
    uint32_t last_reload_generation = shared.reloadGeneration();
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        (void)maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "fc");
        const int64_t ts = bev::nowSteadyNs();
        // Placeholder source; swap with BetaflightMSP adapter poll when serial integration is enabled.
        auto d = bev::ipc::makeAttitudeDesc();
        d.sample_id = sid++;
        d.timestamp_ns = ts;
        d.roll_rad = 0.0F;
        d.pitch_rad = 0.0F;
        d.yaw_rad = 0.0F;
        d.qw = 1.0F;
        (void)q_fc.pushDropOldest(d);
        shared.updateHeartbeat(bev::ipc::RoleId::Fc, static_cast<uint64_t>(ts), d.sample_id, q_fc.dropCount());
        shared.publishLatestFc(d.sample_id, d.timestamp_ns, d.roll_rad, d.pitch_rad, d.yaw_rad);
        shared.recordStageKpi(
            bev::ipc::RoleId::Fc,
            static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
            q_fc.depth(),
            q_fc.dropCount());
        loop_count++;
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_log_ns >= 1000000000LL) {
            std::cerr << "[fc] loops=" << loop_count
                      << " q_fc_to_bev_depth=" << q_fc.depth()
                      << " q_fc_to_bev_dropped=" << q_fc.dropCount()
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}

int runBev(const bev::AppConfig& cfg, const std::string& config_path, const std::string& calib_path) {
    bev::AppConfig cfg_runtime = cfg;
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    std::string err;
    bev::ipc::SharedState shared;
    if (!attachSharedStateWithRetry(shared, names.shared_state, 100, 50, err)) {
        std::cerr << "bev: attach shared state failed: " << err << "\n";
        return 1;
    }
    bev::CameraCalibration calibration;
    cv::Mat K;
    if (!calibration.loadFromFile(calib_path, err)) {
        std::cerr << "bev: calibration load failed, using fallback intrinsics: " << err << "\n";
        const double fx = std::max(1, cfg_runtime.camera.width);
        const double fy = std::max(1, cfg_runtime.camera.height);
        const double cx = static_cast<double>(cfg_runtime.camera.width) * 0.5;
        const double cy = static_cast<double>(cfg_runtime.camera.height) * 0.5;
        K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    } else {
        K = calibration.data().K;
    }
    bev::BEVWarp bev_warp(K);

    bev::ipc::ShmFramePool pool_cam;
    bev::ipc::ShmFramePool pool_bev;
    bev::ipc::ShmFramePool pool_bev_web;
    bev::ipc::TypedShmSpscRing<bev::ipc::FrameDesc> q_cam;
    bev::ipc::TypedShmSpscRing<bev::ipc::AttitudeDesc> q_fc;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> q_bev_track;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> q_bev_web;
    const WebBodyViewMode web_body_mode = parseWebBodyViewMode(cfg_runtime.ipc.web_body_view_mode);

    constexpr int kAttachTries = 600; // 30s at 50ms
    if (!retryResourceInit([&](std::string& e) {
            return pool_cam.attach(names.pool_cam, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_camera), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_camera), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_cam.attach(names.q_cam_to_bev, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_cam_to_bev), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_fc.attach(names.q_fc_to_bev, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_fc_to_bev), e);
        }, kAttachTries, 50, &shared, err)) {
        std::cerr << "bev: attach inputs failed: " << err << "\n";
        return 1;
    }
    if (!pool_bev.create(names.pool_bev, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_bev), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_bev), err) ||
        !q_bev_track.create(names.q_bev_to_track, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_bev_to_track), err) ||
        !q_bev_web.create(names.q_track_to_web, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_track_to_web), err)) {
        std::cerr << "bev: create outputs failed: " << err << "\n";
        return 1;
    }
    if (web_body_mode == WebBodyViewMode::RpOnlyBackend &&
        !pool_bev_web.create(
            names.pool_bev_web,
            static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_bev),
            static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_bev),
            err)) {
        std::cerr << "bev: create pool_bev_web failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Bev, "bev");

    bev::AttitudeSyncBuffer sync_buf(4096);
    bev::SyncPolicy policy{};
    policy.max_attitude_age_ns = static_cast<int64_t>(cfg_runtime.ipc.max_attitude_age_ms) * 1000000LL;
    policy.max_interp_gap_ns = static_cast<int64_t>(cfg_runtime.ipc.max_interp_gap_ms) * 1000000LL;
    policy.startup_min_buffer = static_cast<std::size_t>(cfg_runtime.ipc.startup_min_fc_buffer);

    uint32_t last_reload_generation = shared.reloadGeneration();
    cv::Mat bev_bgr;
    cv::Mat bev_gray;
    cv::Mat bev_body_bgr;
    cv::Mat bev_body_gray;
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        if (maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "bev")) {
            policy.max_attitude_age_ns = static_cast<int64_t>(cfg_runtime.ipc.max_attitude_age_ms) * 1000000LL;
            policy.max_interp_gap_ns = static_cast<int64_t>(cfg_runtime.ipc.max_interp_gap_ms) * 1000000LL;
            policy.startup_min_buffer = static_cast<std::size_t>(cfg_runtime.ipc.startup_min_fc_buffer);
        }
        bev::ipc::AttitudeDesc att{};
        while (q_fc.pop(att)) {
            sync_buf.push(att);
        }

        bev::ipc::FrameDesc f{};
        if (!q_cam.pop(f)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const std::size_t cam_slot = static_cast<std::size_t>(f.frame_slot_id);
        if (!pool_cam.acquireReadSlot(cam_slot, f.generation)) {
            continue;
        }
        const auto synced = sync_buf.resolve(f.timestamp_ns, policy);
        if (!synced.ok) {
            shared.addSyncSample(false, true, static_cast<uint64_t>(policy.max_attitude_age_ns));
            pool_cam.releaseSlot(cam_slot);
            continue;
        }
        shared.addSyncSample(
            synced.interpolated,
            false,
            static_cast<uint64_t>(std::llabs(synced.sync_error_ns)));

        cv::Mat cam = matFromSlot(pool_cam, cam_slot);
        const cv::Matx33d R = bev::composeCameraWorldRotation(
            synced.attitude.roll_rad,
            synced.attitude.pitch_rad,
            synced.attitude.yaw_rad,
            cfg_runtime.camera_mount.negate_yaw,
            cfg_runtime.camera_mount.roll_deg,
            cfg_runtime.camera_mount.pitch_deg,
            cfg_runtime.camera_mount.yaw_deg);
        const float yaw_for_web = static_cast<float>(
            cfg_runtime.camera_mount.negate_yaw ? -synced.attitude.yaw_rad : synced.attitude.yaw_rad);
        bev_warp.warpToBEV(cam, bev_bgr, R);
        cv::cvtColor(bev_bgr, bev_gray, cv::COLOR_BGR2GRAY);

        std::size_t bev_slot = 0;
        if (!pool_bev.acquireWritableSlot(bev_slot)) {
            continue;
        }
        uint32_t gen = 0;
        const bool copied = copyMatToPayload(bev_gray, pool_bev.payloadPtr(bev_slot), pool_bev.payloadBytesPerSlot());
        bool published = false;
        if (copied && pool_bev.publishWrittenSlot(
                bev_slot,
                static_cast<uint32_t>(bev_gray.cols),
                static_cast<uint32_t>(bev_gray.rows),
                static_cast<uint32_t>(bev_gray.type()),
                static_cast<uint32_t>(bev_gray.cols * bev_gray.elemSize()),
                f.timestamp_ns,
                gen)) {
            auto bd = bev::ipc::makeBevFrameDesc();
            bd.frame_slot_id = static_cast<uint32_t>(bev_slot);
            bd.generation = gen;
            bd.width = static_cast<uint32_t>(bev_gray.cols);
            bd.height = static_cast<uint32_t>(bev_gray.rows);
            bd.type = static_cast<uint32_t>(bev_gray.type());
            bd.stride = static_cast<uint32_t>(bev_gray.cols * bev_gray.elemSize());
            bd.timestamp_ns = f.timestamp_ns;
            bd.sequence_id = f.sequence_id;
            bd.reserved0 = packYawRad(yaw_for_web);
            (void)q_bev_track.pushDropOldest(bd);
            if (web_body_mode == WebBodyViewMode::RpOnlyBackend) {
                const cv::Matx33d R_rp_only = bev::composeCameraWorldRotation(
                    synced.attitude.roll_rad,
                    synced.attitude.pitch_rad,
                    0.0,
                    false,
                    cfg_runtime.camera_mount.roll_deg,
                    cfg_runtime.camera_mount.pitch_deg,
                    cfg_runtime.camera_mount.yaw_deg);
                bev_warp.warpToBEV(cam, bev_body_bgr, R_rp_only);
                cv::cvtColor(bev_body_bgr, bev_body_gray, cv::COLOR_BGR2GRAY);
                std::size_t web_slot = 0;
                if (pool_bev_web.acquireWritableSlot(web_slot)) {
                    uint32_t web_gen = 0;
                    const bool web_copied = copyMatToPayload(bev_body_gray, pool_bev_web.payloadPtr(web_slot), pool_bev_web.payloadBytesPerSlot());
                    if (web_copied && pool_bev_web.publishWrittenSlot(
                            web_slot,
                            static_cast<uint32_t>(bev_body_gray.cols),
                            static_cast<uint32_t>(bev_body_gray.rows),
                            static_cast<uint32_t>(bev_body_gray.type()),
                            static_cast<uint32_t>(bev_body_gray.cols * bev_body_gray.elemSize()),
                            f.timestamp_ns,
                            web_gen)) {
                        auto wb = bd;
                        wb.frame_slot_id = static_cast<uint32_t>(web_slot);
                        wb.generation = web_gen;
                        wb.width = static_cast<uint32_t>(bev_body_gray.cols);
                        wb.height = static_cast<uint32_t>(bev_body_gray.rows);
                        wb.type = static_cast<uint32_t>(bev_body_gray.type());
                        wb.stride = static_cast<uint32_t>(bev_body_gray.cols * bev_body_gray.elemSize());
                        wb.reserved0 = packYawRad(0.0F);
                        (void)q_bev_web.pushDropOldest(wb);
                    } else {
                        pool_bev_web.releaseSlot(web_slot);
                    }
                }
            } else {
                (void)q_bev_web.pushDropOldest(bd);
            }
            shared.updateHeartbeat(bev::ipc::RoleId::Bev, static_cast<uint64_t>(f.timestamp_ns), f.sequence_id, q_bev_track.dropCount());
            shared.publishLatestBev(
                bd.frame_slot_id,
                bd.generation,
                bd.width,
                bd.height,
                bd.type,
                bd.stride,
                bd.sequence_id,
                bd.timestamp_ns,
                bd.reserved0);
            shared.recordStageKpi(
                bev::ipc::RoleId::Bev,
                static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
                q_cam.depth(),
                q_bev_track.dropCount());
            published = true;
        }
        pool_cam.releaseSlot(cam_slot);
        if (!published) {
            pool_bev.releaseSlot(bev_slot);
        }
        loop_count++;
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_log_ns >= 1000000000LL) {
            std::cerr << "[bev] loops=" << loop_count
                      << " q_cam_to_bev_depth=" << q_cam.depth()
                      << " q_fc_to_bev_depth=" << q_fc.depth()
                      << " q_bev_to_track_depth=" << q_bev_track.depth()
                      << " q_bev_to_track_dropped=" << q_bev_track.dropCount()
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns;
        }
    }
    return 0;
}

int runTrack(const bev::AppConfig& cfg, const std::string& config_path) {
    bev::AppConfig cfg_runtime = cfg;
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    std::string err;
    bev::ipc::SharedState shared;
    if (!attachSharedStateWithRetry(shared, names.shared_state, 100, 50, err)) {
        std::cerr << "track: attach shared state failed: " << err << "\n";
        return 1;
    }
    bev::ipc::ShmFramePool pool_bev;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> q_bev;
    bev::ipc::TypedShmSpscRing<bev::ipc::TrackResultDesc> q_track_map;
    constexpr int kAttachTries = 600; // 30s at 50ms
    if (!retryResourceInit([&](std::string& e) {
            return pool_bev.attach(names.pool_bev, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_bev), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_bev), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_bev.attach(names.q_bev_to_track, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_bev_to_track), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_track_map.create(names.q_track_to_map, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_track_to_map), e);
        }, 10, 50, &shared, err)) {
        std::cerr << "track: ring/pool init failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Track, "track");

    auto clampScale = [](float s) { return std::max(0.1F, std::min(1.0F, s)); };
    auto scaledOdd = [](int base, float scale) {
        int v = std::max(3, static_cast<int>(std::lround(static_cast<float>(base) * scale)));
        if ((v % 2) == 0) {
            v += 1;
        }
        return v;
    };
    auto scaledAtLeast = [](int base, float scale, int min_v) {
        return std::max(min_v, static_cast<int>(std::lround(static_cast<float>(base) * scale)));
    };

    float tracking_scale = clampScale(cfg_runtime.tracking.processing_scale);
    int scaled_template_size = scaledOdd(cfg_runtime.tracking.template_size, tracking_scale);
    int scaled_coarse_crop = scaledAtLeast(cfg_runtime.tracking.coarse_crop_size, tracking_scale, scaled_template_size);
    int scaled_fine_crop = scaledAtLeast(cfg_runtime.tracking.fine_crop_size, tracking_scale, scaled_template_size);
    bev::TrackManager track_manager(cfg_runtime.tracking.max_distance_from_keypoint_center * tracking_scale);
    bev::TrackManager::RecoveryState recovery{};
    std::vector<bev::TrackPoint> tracks;
    tracks.reserve(static_cast<std::size_t>(std::max(1, cfg_runtime.tracking.num_keypoints)));
    cv::Mat prev_bev_gray;
    cv::Mat bev_work_gray;
    cv::Mat prev_work_gray;
    int64_t prev_ts = 0;
    int next_track_id = 0;

    auto reinitializeTracks = [&](const cv::Mat& gray) {
        std::vector<cv::Point2f> seeds;
        bev::KeypointInitializer::initializeShiTomasiInPlace(
            gray, cfg_runtime.tracking.num_keypoints, seeds, 0.01, 8.0);
        if (seeds.empty()) {
            const int rows = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(std::max(1, cfg_runtime.tracking.num_keypoints)))));
            const int cols = std::max(1, cfg_runtime.tracking.num_keypoints / rows);
            seeds = bev::KeypointInitializer::initializeGrid(gray.size(), rows, cols, 20);
        }
        tracks.clear();
        tracks.reserve(seeds.size());
        for (const auto& p : seeds) {
            bev::TrackPoint t;
            t.id = next_track_id++;
            t.pos = p;
            t.vel_px_s = cv::Point2f(0.0F, 0.0F);
            t.confidence = 1.0F;
            t.valid = true;
            tracks.push_back(t);
        }
    };

    uint32_t last_reload_generation = shared.reloadGeneration();
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        if (maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "track")) {
            tracking_scale = clampScale(cfg_runtime.tracking.processing_scale);
            scaled_template_size = scaledOdd(cfg_runtime.tracking.template_size, tracking_scale);
            scaled_coarse_crop = scaledAtLeast(cfg_runtime.tracking.coarse_crop_size, tracking_scale, scaled_template_size);
            scaled_fine_crop = scaledAtLeast(cfg_runtime.tracking.fine_crop_size, tracking_scale, scaled_template_size);
            track_manager = bev::TrackManager(cfg_runtime.tracking.max_distance_from_keypoint_center * tracking_scale);
            tracks.clear();
            prev_work_gray.release();
            prev_bev_gray.release();
            prev_ts = 0;
        }
        bev::ipc::BevFrameDesc bd{};
        if (!q_bev.pop(bd)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const std::size_t slot = static_cast<std::size_t>(bd.frame_slot_id);
        if (!pool_bev.acquireReadSlot(slot, bd.generation)) {
            continue;
        }
        cv::Mat bev_gray = matFromSlot(pool_bev, slot);
        if (tracking_scale < 0.999F) {
            const int w = std::max(8, static_cast<int>(std::lround(static_cast<float>(bev_gray.cols) * tracking_scale)));
            const int h = std::max(8, static_cast<int>(std::lround(static_cast<float>(bev_gray.rows) * tracking_scale)));
            cv::resize(bev_gray, bev_work_gray, cv::Size(w, h), 0.0, 0.0, cv::INTER_AREA);
        } else {
            bev_work_gray = bev_gray;
        }

        if (tracks.empty() || prev_work_gray.empty() || prev_ts <= 0) {
            reinitializeTracks(bev_work_gray);
            prev_work_gray = bev_work_gray.clone();
            prev_bev_gray = bev_gray.clone();
            prev_ts = bd.timestamp_ns;
            auto td = bev::ipc::makeTrackResultDesc();
            td.frame_slot_id = bd.frame_slot_id;
            td.generation = bd.generation;
            td.timestamp_ns = bd.timestamp_ns;
            td.sequence_id = bd.sequence_id;
            td.valid_track_count = static_cast<uint32_t>(bev::TrackManager::countValidTracks(tracks));
            td.vx_px_s = 0.0F;
            td.vy_px_s = 0.0F;
            td.omega_rad_s = 0.0F;
            td.inlier_ratio = 1.0F;
            td.confidence = bev::TrackManager::meanConfidence(tracks);
            td.reproj_rmse = 0.0F;
            (void)q_track_map.pushDropOldest(td);
            shared.updateHeartbeat(bev::ipc::RoleId::Track, static_cast<uint64_t>(bd.timestamp_ns), bd.sequence_id, q_track_map.dropCount());
            shared.recordStageKpi(
                bev::ipc::RoleId::Track,
                static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
                q_bev.depth(),
                q_track_map.dropCount());
            pool_bev.releaseSlot(slot);
            continue;
        }

        const float dt_s = (bd.timestamp_ns > prev_ts)
            ? static_cast<float>(bd.timestamp_ns - prev_ts) / 1e9F
            : 0.0F;
        const int templ_half = scaled_template_size / 2;
        const int coarse_half = scaled_coarse_crop / 2;
        const float min_match_score = 0.35F;

        float sum_vx = 0.0F;
        float sum_vy = 0.0F;
        float sum_w = 0.0F;
        int matched = 0;
        std::vector<cv::Point2f> matched_prev;
        std::vector<cv::Point2f> matched_curr;
        std::vector<float> matched_w;
        matched_prev.reserve(tracks.size());
        matched_curr.reserve(tracks.size());
        matched_w.reserve(tracks.size());

        for (auto& tr : tracks) {
            if (!tr.valid || dt_s <= 1e-6F) {
                tr.valid = false;
                tr.confidence = 0.0F;
                continue;
            }

            const int px = static_cast<int>(std::lround(tr.pos.x));
            const int py = static_cast<int>(std::lround(tr.pos.y));
            const cv::Rect templ_rect(
                px - templ_half,
                py - templ_half,
                scaled_template_size,
                scaled_template_size);
            if (templ_rect.x < 0 ||
                templ_rect.y < 0 ||
                templ_rect.x + templ_rect.width > prev_work_gray.cols ||
                templ_rect.y + templ_rect.height > prev_work_gray.rows) {
                tr.valid = false;
                tr.confidence = 0.0F;
                continue;
            }

            const cv::Mat templ = prev_work_gray(templ_rect);
            const cv::Point2f predicted = track_manager.predictSearchCenter(tr, dt_s);
            const cv::Rect coarse_window(
                static_cast<int>(std::lround(predicted.x)) - coarse_half,
                static_cast<int>(std::lround(predicted.y)) - coarse_half,
                scaled_coarse_crop,
                scaled_coarse_crop);
            const auto m = bev::coarseFineMatch(
                bev_work_gray,
                templ,
                coarse_window,
                scaled_fine_crop,
                2);
            if (!m.valid || m.score < min_match_score) {
                tr.valid = false;
                tr.confidence = 0.0F;
                continue;
            }

            const cv::Point2f prev_pos = tr.pos;
            tr.pos = cv::Point2f(
                m.global_top_left.x + static_cast<float>(templ_half),
                m.global_top_left.y + static_cast<float>(templ_half));
            tr.vel_px_s = cv::Point2f(
                (tr.pos.x - prev_pos.x) / dt_s,
                (tr.pos.y - prev_pos.y) / dt_s);
            tr.confidence = std::max(0.0F, std::min(1.0F, m.score));
            tr.valid = true;

            const float w = std::max(0.1F, tr.confidence);
            sum_vx += tr.vel_px_s.x * w;
            sum_vy += tr.vel_px_s.y * w;
            sum_w += w;
            ++matched;
            matched_prev.push_back(prev_pos);
            matched_curr.push_back(tr.pos);
            matched_w.push_back(w);
        }

        float vx = 0.0F;
        float vy = 0.0F;
        if (sum_w > 1e-6F) {
            vx = sum_vx / sum_w;
            vy = sum_vy / sum_w;
        }
        const float omega = bev::TrackManager::estimateOmegaRadPerSec(matched_prev, matched_curr, dt_s, &matched_w);

        recovery = bev::TrackManager::updateRecoveryState(
            recovery,
            tracks,
            cfg_runtime.tracking.min_valid_tracks,
            0.35F,
            5);
        shared.setPauseMapUpdates(recovery.map_updates_paused);
        shared.publishTrackerWatchdog(
            static_cast<uint64_t>(std::max(0, recovery.low_confidence_streak)),
            recovery.should_reinitialize);
        if (recovery.should_reinitialize) {
            reinitializeTracks(bev_work_gray);
        }

        prev_work_gray = bev_work_gray.clone();
        prev_bev_gray = bev_gray.clone();
        prev_ts = bd.timestamp_ns;

        auto td = bev::ipc::makeTrackResultDesc();
        td.frame_slot_id = bd.frame_slot_id;
        td.generation = bd.generation;
        td.timestamp_ns = bd.timestamp_ns;
        td.sequence_id = bd.sequence_id;
        td.valid_track_count = static_cast<uint32_t>(bev::TrackManager::countValidTracks(tracks));
        td.vx_px_s = vx * (1.0F / tracking_scale);
        td.vy_px_s = vy * (1.0F / tracking_scale);
        td.omega_rad_s = omega;
        td.inlier_ratio = (tracks.empty())
            ? 0.0F
            : static_cast<float>(matched) / static_cast<float>(tracks.size());
        td.confidence = bev::TrackManager::meanConfidence(tracks);
        td.reproj_rmse = 0.0F;
        (void)q_track_map.pushDropOldest(td);
        shared.updateHeartbeat(bev::ipc::RoleId::Track, static_cast<uint64_t>(bd.timestamp_ns), bd.sequence_id, q_track_map.dropCount());
        shared.recordStageKpi(
            bev::ipc::RoleId::Track,
            static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
            q_bev.depth(),
            q_track_map.dropCount());
        pool_bev.releaseSlot(slot);
        loop_count++;
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_log_ns >= 1000000000LL) {
            std::cerr << "[track] loops=" << loop_count
                      << " q_bev_to_track_depth=" << q_bev.depth()
                      << " q_track_to_map_depth=" << q_track_map.depth()
                      << " q_track_to_map_dropped=" << q_track_map.dropCount()
                      << " scale=" << tracking_scale
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns;
        }
    }
    return 0;
}

int runMap(const bev::AppConfig& cfg, const std::string& config_path) {
    bev::AppConfig cfg_runtime = cfg;
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    std::string err;
    bev::ipc::SharedState shared;
    if (!attachSharedStateWithRetry(shared, names.shared_state, 100, 50, err)) {
        std::cerr << "map: attach shared state failed: " << err << "\n";
        return 1;
    }
    bev::ipc::ShmFramePool pool_bev;
    bev::ipc::ShmFramePool pool_map;
    bev::ipc::TypedShmSpscRing<bev::ipc::TrackResultDesc> q_track_map;
    bev::ipc::TypedShmSpscRing<bev::ipc::MapFrameDesc> q_map_web;
    constexpr int kAttachTries = 600; // 30s at 50ms
    if (!retryResourceInit([&](std::string& e) {
            return pool_bev.attach(names.pool_bev, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_bev), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_bev), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_track_map.attach(names.q_track_to_map, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_track_to_map), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return pool_map.create(names.pool_map, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_map), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_map), e);
        }, 10, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_map_web.create(names.q_map_to_web, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_map_to_web), e);
        }, 10, 50, &shared, err)) {
        std::cerr << "map: ring/pool init failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Map, "map");

    bev::MinimapBuilder minimap;
    if (!minimap.initialize(cfg_runtime.map.initial_width, cfg_runtime.map.initial_height)) {
        std::cerr << "map: minimap init failed\n";
        return 1;
    }

    uint32_t last_reload_generation = shared.reloadGeneration();
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    uint64_t map_pause_streak = 0;
    bool last_map_unstable = false;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        if (maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "map")) {
            // Keep current map content; only runtime behavior uses new config.
        }
        bev::ipc::TrackResultDesc td{};
        if (!q_track_map.pop(td)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const std::size_t bev_slot = static_cast<std::size_t>(td.frame_slot_id);
        if (!pool_bev.acquireReadSlot(bev_slot, td.generation)) {
            continue;
        }
        cv::Mat bev_gray = matFromSlot(pool_bev, bev_slot);
        const bool paused = shared.pauseMapUpdates();
        if (paused) {
            map_pause_streak += 1;
        } else {
            map_pause_streak = 0;
        }
        const bool map_unstable = paused || (td.confidence < 0.35F);
        shared.publishMapWatchdog(map_pause_streak, map_unstable && !last_map_unstable);
        last_map_unstable = map_unstable;
        if (!paused) {
            minimap.addFrame(bev_gray, cv::Mat::eye(3, 3, CV_64F));
        }
        pool_bev.releaseSlot(bev_slot);

        cv::Mat map_gray = minimap.canvas();
        std::size_t map_slot = 0;
        if (!pool_map.acquireWritableSlot(map_slot)) {
            continue;
        }
        uint32_t map_gen = 0;
        const bool copied = copyMatToPayload(map_gray, pool_map.payloadPtr(map_slot), pool_map.payloadBytesPerSlot());
        bool published = false;
        if (copied && pool_map.publishWrittenSlot(
                map_slot,
                static_cast<uint32_t>(map_gray.cols),
                static_cast<uint32_t>(map_gray.rows),
                static_cast<uint32_t>(map_gray.type()),
                static_cast<uint32_t>(map_gray.cols * map_gray.elemSize()),
                td.timestamp_ns,
                map_gen)) {
            auto md = bev::ipc::makeMapFrameDesc();
            md.frame_slot_id = static_cast<uint32_t>(map_slot);
            md.generation = map_gen;
            md.sequence_id = td.sequence_id;
            md.timestamp_ns = td.timestamp_ns;
            md.map_width = static_cast<uint32_t>(map_gray.cols);
            md.map_height = static_cast<uint32_t>(map_gray.rows);
            md.map_confidence = td.confidence;
            md.drift_px = 0.0F;
            md.coverage_ratio = static_cast<float>(minimap.stats().coverage_ratio);
            md.map_paused = paused ? 1U : 0U;
            (void)q_map_web.pushDropOldest(md);
            shared.updateHeartbeat(bev::ipc::RoleId::Map, static_cast<uint64_t>(td.timestamp_ns), td.sequence_id, q_map_web.dropCount());
            shared.publishLatestMap(
                md.frame_slot_id,
                md.generation,
                static_cast<uint32_t>(map_gray.cols),
                static_cast<uint32_t>(map_gray.rows),
                static_cast<uint32_t>(map_gray.type()),
                static_cast<uint32_t>(map_gray.cols * map_gray.elemSize()),
                md.sequence_id,
                md.timestamp_ns,
                md.reserved0);
            shared.recordStageKpi(
                bev::ipc::RoleId::Map,
                static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
                q_track_map.depth(),
                q_map_web.dropCount());
            published = true;
        }
        if (!published) {
            pool_map.releaseSlot(map_slot);
        }
        loop_count++;
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_log_ns >= 1000000000LL) {
            std::cerr << "[map] loops=" << loop_count
                      << " q_track_to_map_depth=" << q_track_map.depth()
                      << " q_map_to_web_depth=" << q_map_web.depth()
                      << " q_map_to_web_dropped=" << q_map_web.dropCount()
                      << " paused=" << (shared.pauseMapUpdates() ? 1 : 0)
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns;
        }
    }
    return 0;
}

int runWeb(const bev::AppConfig& cfg, const std::string& config_path) {
    bev::AppConfig cfg_runtime = cfg;
    const WebBodyViewMode web_body_mode = parseWebBodyViewMode(cfg_runtime.ipc.web_body_view_mode);
    const auto names = bev::ipc::makePipelineNames(cfg_runtime.ipc.shm_namespace);
    std::string err;
    bev::ipc::SharedState shared;
    if (!attachSharedStateWithRetry(shared, names.shared_state, 100, 50, err)) {
        std::cerr << "web: attach shared state failed: " << err << "\n";
        return 1;
    }
    bev::ipc::ShmFramePool pool_bev;
    bev::ipc::ShmFramePool pool_map;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> q_bev_web;
    bev::ipc::TypedShmSpscRing<bev::ipc::MapFrameDesc> q_map_web;
    constexpr int kAttachTries = 600; // 30s at 50ms
    if (!retryResourceInit([&](std::string& e) {
            const std::string& pool_name =
                (web_body_mode == WebBodyViewMode::RpOnlyBackend) ? names.pool_bev_web : names.pool_bev;
            return pool_bev.attach(pool_name, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_bev), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_bev), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return pool_map.attach(names.pool_map, static_cast<std::size_t>(cfg_runtime.ipc.frame_pool_slots_map), static_cast<std::size_t>(cfg_runtime.ipc.frame_payload_bytes_map), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_bev_web.attach(names.q_track_to_web, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_track_to_web), e);
        }, kAttachTries, 50, &shared, err) ||
        !retryResourceInit([&](std::string& e) {
            return q_map_web.attach(names.q_map_to_web, static_cast<uint32_t>(cfg_runtime.ipc.ring_capacity_map_to_web), e);
        }, kAttachTries, 50, &shared, err)) {
        std::cerr << "web: ring/pool init failed: " << err << "\n";
        return 1;
    }
    shared.markReady(bev::ipc::RoleId::Web, "web");

    bev::MJPEGServer server;
    if (!server.start(8080)) {
        std::cerr << "web: failed to start server on 8080\n";
        return 1;
    }
    server.addRoute("/stream/main");
    server.addRoute("/stream/map");
    server.addTextRoute("/meta/main", "application/json");
    int jpeg_quality = cfg_runtime.ipc.web_jpeg_quality_default;
    uint64_t last_bev_drop_count = 0;
    uint64_t last_map_drop_count = 0;
    int64_t last_quality_adjust_ns = bev::nowSteadyNs();

    bev::ipc::UnixControlServer ctl_server;
    if (!ctl_server.start(names.control_socket, [&shared, &jpeg_quality](const std::string& req) -> std::string {
            const std::string clean = req;
            if (clean.find("pause-map") != std::string::npos) {
                shared.setPauseMapUpdates(true);
                return "OK pause-map\n";
            }
            if (clean.find("resume-map") != std::string::npos) {
                shared.setPauseMapUpdates(false);
                return "OK resume-map\n";
            }
            if (clean.find("reload-config") != std::string::npos) {
                const uint32_t g = shared.requestReload();
                return "OK reload-config generation=" + std::to_string(g) + "\n";
            }
            if (clean.find("shutdown") != std::string::npos) {
                shared.requestShutdown();
                return "OK shutdown\n";
            }
            if (clean.find("health") != std::string::npos) {
                std::ostringstream oss;
                const auto* block = shared.block();
                oss << "OK health pause_map=" << (shared.pauseMapUpdates() ? 1 : 0)
                    << " shutdown=" << (shared.shutdownRequested() ? 1 : 0)
                    << " reload_generation=" << shared.reloadGeneration()
                    << " protocol_mismatch=" << (shared.protocolMismatch() ? 1 : 0)
                    << " jpeg_quality=" << jpeg_quality
                    << "\n";
                if (block != nullptr) {
                    const auto sync_samples = block->sync_kpi.sync_samples.load(std::memory_order_acquire);
                    const auto interp_samples = block->sync_kpi.interpolated_samples.load(std::memory_order_acquire);
                    const auto stale_samples = block->sync_kpi.stale_attitude_count.load(std::memory_order_acquire);
                    const auto sync_err_acc = block->sync_kpi.sync_error_abs_ns_accum.load(std::memory_order_acquire);
                    const auto sync_err_max = block->sync_kpi.sync_error_abs_ns_max.load(std::memory_order_acquire);
                    const double sync_err_mean_us = (sync_samples > 0)
                        ? (static_cast<double>(sync_err_acc) / static_cast<double>(sync_samples)) / 1000.0
                        : 0.0;
                    const double interp_ratio = (sync_samples > 0)
                        ? static_cast<double>(interp_samples) / static_cast<double>(sync_samples)
                        : 0.0;
                    oss << "sync_kpi samples=" << sync_samples
                        << " stale=" << stale_samples
                        << " interp_ratio=" << interp_ratio
                        << " mean_abs_error_us=" << sync_err_mean_us
                        << " max_abs_error_us=" << (static_cast<double>(sync_err_max) / 1000.0)
                        << "\n";
                    for (uint32_t i = 0; i < static_cast<uint32_t>(bev::ipc::RoleId::Count); ++i) {
                        const auto& role = block->roles[i];
                        const char* fallback = (i == 0U) ? "camera" :
                                               (i == 1U) ? "fc" :
                                               (i == 2U) ? "bev" :
                                               (i == 3U) ? "track" :
                                               (i == 4U) ? "map" : "web";
                        const char* role_name = (role.name[0] != '\0') ? role.name : fallback;
                        oss << role_name
                            << " ready=" << role.ready.load(std::memory_order_acquire)
                            << " heartbeat_ns=" << role.heartbeat_ns.load(std::memory_order_acquire)
                            << " seq=" << role.sequence_id.load(std::memory_order_acquire)
                            << " dropped=" << role.dropped.load(std::memory_order_acquire)
                            << "\n";
                        const auto& stage = block->stage_kpi[i];
                        oss << "stage_" << role_name
                            << " samples=" << stage.samples.load(std::memory_order_acquire)
                            << " latency_last_us=" << (stage.latency_last_ns.load(std::memory_order_acquire) / 1000U)
                            << " latency_mean_us=" << (stage.latency_mean_ns.load(std::memory_order_acquire) / 1000U)
                            << " latency_max_us=" << (stage.latency_max_ns.load(std::memory_order_acquire) / 1000U)
                            << " queue_depth=" << stage.queue_depth.load(std::memory_order_acquire)
                            << " dropped=" << stage.dropped.load(std::memory_order_acquire)
                            << "\n";
                    }
                    oss << "snapshot_camera valid=" << block->latest_camera.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_camera.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_camera.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_camera.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_camera.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_fc valid=" << block->latest_fc.valid.load(std::memory_order_acquire)
                        << " sample_id=" << block->latest_fc.sample_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_fc.timestamp_ns.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_bev valid=" << block->latest_bev.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_bev.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_bev.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_bev.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_bev.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "snapshot_map valid=" << block->latest_map.valid.load(std::memory_order_acquire)
                        << " seq=" << block->latest_map.sequence_id.load(std::memory_order_acquire)
                        << " ts_ns=" << block->latest_map.timestamp_ns.load(std::memory_order_acquire)
                        << " slot=" << block->latest_map.frame_slot_id.load(std::memory_order_acquire)
                        << " gen=" << block->latest_map.generation.load(std::memory_order_acquire)
                        << "\n";
                    oss << "thermal valid=" << block->thermal.valid.load(std::memory_order_acquire)
                        << " throttled=" << block->thermal.throttled.load(std::memory_order_acquire)
                        << " temp_mC=" << block->thermal.temp_milli_c.load(std::memory_order_acquire)
                        << " fps_cap=" << block->thermal.fps_cap.load(std::memory_order_acquire)
                        << "\n";
                    oss << "watchdog msp_stale_total=" << block->watchdog.msp_stale_total.load(std::memory_order_acquire)
                        << " msp_stale_streak=" << block->watchdog.msp_stale_streak.load(std::memory_order_acquire)
                        << " tracker_reinit_total=" << block->watchdog.tracker_reinit_total.load(std::memory_order_acquire)
                        << " tracker_failure_streak=" << block->watchdog.tracker_failure_streak.load(std::memory_order_acquire)
                        << " map_pause_streak=" << block->watchdog.map_pause_streak.load(std::memory_order_acquire)
                        << " map_instability_total=" << block->watchdog.map_instability_total.load(std::memory_order_acquire)
                        << " alert_msp_stale=" << block->watchdog.alert_msp_stale.load(std::memory_order_acquire)
                        << " alert_tracker_failure=" << block->watchdog.alert_tracker_failure.load(std::memory_order_acquire)
                        << " alert_map_instability=" << block->watchdog.alert_map_instability.load(std::memory_order_acquire)
                        << "\n";
                }
                return oss.str();
            }
            return "ERR unknown-command\n";
        }, err)) {
        std::cerr << "web: failed to start control socket: " << err << "\n";
        return 1;
    }

    uint32_t last_reload_generation = shared.reloadGeneration();
    int main_frames_this_sec = 0;
    int main_fps = 0;
    int64_t main_fps_window_start_ns = bev::nowSteadyNs();
    float latest_yaw_rad = 0.0F;
    uint64_t last_bev_seq_snapshot = 0;
    uint64_t last_map_seq_snapshot = 0;
    int64_t last_log_ns = bev::nowSteadyNs();
    uint64_t loop_count = 0;
    while (g_running.load()) {
        const int64_t loop_start_ns = bev::nowSteadyNs();
        if (shared.shutdownRequested()) {
            break;
        }
        (void)maybeReloadConfig(cfg_runtime, config_path, shared, last_reload_generation, "web");
        const int64_t now_ns = bev::nowSteadyNs();
        if (now_ns - last_quality_adjust_ns >= 1000000000LL) {
            const uint64_t bev_drops = q_bev_web.dropCount();
            const uint64_t map_drops = q_map_web.dropCount();
            const uint64_t delta_drops = (bev_drops - last_bev_drop_count) + (map_drops - last_map_drop_count);
            if (delta_drops > static_cast<uint64_t>(cfg_runtime.ipc.web_drop_threshold_per_sec)) {
                jpeg_quality = std::max(cfg_runtime.ipc.web_jpeg_quality_min, jpeg_quality - cfg_runtime.ipc.web_quality_step);
            } else if (delta_drops == 0U && jpeg_quality < cfg_runtime.ipc.web_jpeg_quality_default) {
                jpeg_quality = std::min(cfg_runtime.ipc.web_jpeg_quality_default, jpeg_quality + cfg_runtime.ipc.web_quality_step);
            }
            last_bev_drop_count = bev_drops;
            last_map_drop_count = map_drops;
            last_quality_adjust_ns = now_ns;
        }
        bev::ipc::BevFrameDesc bd{};
        bool got_bev = false;
        while (q_bev_web.pop(bd)) {
            got_bev = true;
            const std::size_t slot = static_cast<std::size_t>(bd.frame_slot_id);
            if (!pool_bev.acquireReadSlot(slot, bd.generation)) {
                continue;
            }
            cv::Mat gray = matFromSlot(pool_bev, slot);
            // Keep JPEG path grayscale to avoid per-frame color-conversion allocation.
            cv::Mat gray_overlay = gray;
            const int64_t now2 = bev::nowSteadyNs();
            main_frames_this_sec += 1;
            if (now2 - main_fps_window_start_ns >= 1000000000LL) {
                main_fps = main_frames_this_sec;
                main_frames_this_sec = 0;
                main_fps_window_start_ns = now2;
            }
            cv::putText(
                gray_overlay,
                "FPS: " + std::to_string(main_fps),
                cv::Point(10, 24),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(255),
                2,
                cv::LINE_AA);
            latest_yaw_rad = unpackYawRad(bd.reserved0);
            server.updateGrayFrame("/stream/main", gray_overlay, jpeg_quality);
            pool_bev.releaseSlot(slot);
            last_bev_seq_snapshot = bd.sequence_id;
        }
        if (!got_bev) {
            const auto* block = shared.block();
            if (block != nullptr && block->latest_bev.valid.load(std::memory_order_acquire) != 0U) {
                const uint64_t seq = block->latest_bev.sequence_id.load(std::memory_order_acquire);
                if (seq > last_bev_seq_snapshot) {
                    const uint32_t slot_id = block->latest_bev.frame_slot_id.load(std::memory_order_acquire);
                    const uint32_t gen = block->latest_bev.generation.load(std::memory_order_acquire);
                    const std::size_t slot = static_cast<std::size_t>(slot_id);
                    if (pool_bev.acquireReadSlot(slot, gen)) {
                        cv::Mat gray = matFromSlot(pool_bev, slot);
                        cv::Mat gray_overlay = gray;
                        cv::putText(
                            gray_overlay,
                            "FPS: " + std::to_string(main_fps),
                            cv::Point(10, 24),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.65,
                            cv::Scalar(255),
                            2,
                            cv::LINE_AA);
                        latest_yaw_rad = unpackYawRad(block->latest_bev.aux0.load(std::memory_order_acquire));
                        server.updateGrayFrame("/stream/main", gray_overlay, jpeg_quality);
                        pool_bev.releaseSlot(slot);
                        last_bev_seq_snapshot = seq;
                    }
                }
            }
        }
        bev::ipc::MapFrameDesc md{};
        bool got_map = false;
        while (q_map_web.pop(md)) {
            got_map = true;
            const std::size_t slot = static_cast<std::size_t>(md.frame_slot_id);
            if (!pool_map.acquireReadSlot(slot, md.generation)) {
                continue;
            }
            cv::Mat gray = matFromSlot(pool_map, slot);
            server.updateGrayFrame("/stream/map", gray, jpeg_quality);
            pool_map.releaseSlot(slot);
            last_map_seq_snapshot = md.sequence_id;
        }
        if (!got_map) {
            const auto* block = shared.block();
            if (block != nullptr && block->latest_map.valid.load(std::memory_order_acquire) != 0U) {
                const uint64_t seq = block->latest_map.sequence_id.load(std::memory_order_acquire);
                if (seq > last_map_seq_snapshot) {
                    const uint32_t slot_id = block->latest_map.frame_slot_id.load(std::memory_order_acquire);
                    const uint32_t gen = block->latest_map.generation.load(std::memory_order_acquire);
                    const std::size_t slot = static_cast<std::size_t>(slot_id);
                    if (pool_map.acquireReadSlot(slot, gen)) {
                        cv::Mat gray = matFromSlot(pool_map, slot);
                        server.updateGrayFrame("/stream/map", gray, jpeg_quality);
                        pool_map.releaseSlot(slot);
                        last_map_seq_snapshot = seq;
                    }
                }
            }
        }
        {
            std::ostringstream oss;
            oss << "{\"mode\":\"" << cfg_runtime.ipc.web_body_view_mode
                << "\",\"yaw_rad\":" << latest_yaw_rad
                << ",\"fps\":" << main_fps
                << "}";
            server.updateText("/meta/main", oss.str());
        }
        shared.updateHeartbeat(bev::ipc::RoleId::Web, static_cast<uint64_t>(bev::nowSteadyNs()), 0U, 0U);
        const uint64_t q_depth = q_bev_web.depth() + q_map_web.depth();
        const uint64_t q_drop = q_bev_web.dropCount() + q_map_web.dropCount();
        shared.recordStageKpi(
            bev::ipc::RoleId::Web,
            static_cast<uint64_t>(std::max<int64_t>(0, bev::nowSteadyNs() - loop_start_ns)),
            q_depth,
            q_drop);
        loop_count++;
        const int64_t now_ns_log = bev::nowSteadyNs();
        if (now_ns_log - last_log_ns >= 1000000000LL) {
            std::cerr << "[web] loops=" << loop_count
                      << " q_bev_to_web_depth=" << q_bev_web.depth()
                      << " q_map_to_web_depth=" << q_map_web.depth()
                      << " q_bev_to_web_dropped=" << q_bev_web.dropCount()
                      << " q_map_to_web_dropped=" << q_map_web.dropCount()
                      << " jpeg_quality=" << jpeg_quality
                      << "\n";
            loop_count = 0;
            last_log_ns = now_ns_log;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_runtime.ipc.web_dashboard_period_ms));
    }
    ctl_server.stop();
    server.stop();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSigInt);
    if (argc < 2) {
        std::cerr << "usage: bev_mp_node <role> [config_path] [calib_path]\n";
        return 1;
    }
    const std::string role = argv[1];
    const std::string config_path = (argc > 2) ? argv[2] : "config/config.yaml";
    const std::string calib_path = (argc > 3) ? argv[3] : "config/camera_calibration.yaml";

    bev::AppConfig cfg;
    std::string err;
    if (!bev::loadConfig(config_path, cfg, err)) {
        std::cerr << "config load failed: " << err << "\n";
        return 1;
    }

    applyRoleScheduling(role);

    if (role == "camera") return runCamera(cfg, config_path, calib_path);
    if (role == "fc") return runFc(cfg, config_path);
    if (role == "bev") return runBev(cfg, config_path, calib_path);
    if (role == "track") return runTrack(cfg, config_path);
    if (role == "map") return runMap(cfg, config_path);
    if (role == "web") return runWeb(cfg, config_path);

    std::cerr << "unknown role: " << role << "\n";
    return 1;
}
