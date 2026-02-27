#include "ipc/shared_state.hpp"

#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace bev::ipc {

namespace {
constexpr uint32_t kSharedStateMagic = 0x42565354U;  // BVST
constexpr uint32_t kSharedStateVersion = 2U;

uint32_t packFloatBits(float v) {
    uint32_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(v), "float/u32 size mismatch");
    std::memcpy(&bits, &v, sizeof(bits));
    return bits;
}

void resetWatchdog(WatchdogSnapshot& w) {
    w.msp_stale_total.store(0U, std::memory_order_release);
    w.msp_stale_streak.store(0U, std::memory_order_release);
    w.tracker_reinit_total.store(0U, std::memory_order_release);
    w.tracker_failure_streak.store(0U, std::memory_order_release);
    w.map_pause_streak.store(0U, std::memory_order_release);
    w.map_instability_total.store(0U, std::memory_order_release);
    w.alert_msp_stale.store(0U, std::memory_order_release);
    w.alert_tracker_failure.store(0U, std::memory_order_release);
    w.alert_map_instability.store(0U, std::memory_order_release);
}
}

SharedState::~SharedState() {
    close();
}

bool SharedState::mapInternal(int fd, std::string& error) {
#ifdef __linux__
    void* ptr = mmap(nullptr, sizeof(SharedStateBlock), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        error = "mmap failed";
        return false;
    }
    block_ = static_cast<SharedStateBlock*>(ptr);
    return true;
#else
    (void)fd;
    error = "SharedState requires Linux/POSIX shared memory";
    return false;
#endif
}

bool SharedState::create(const std::string& name, std::string& error) {
    close();
#ifdef __linux__
    name_ = name;
    bool created_new = false;
    fd_ = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd_ >= 0) {
        created_new = true;
        owner_ = true;
        if (ftruncate(fd_, static_cast<off_t>(sizeof(SharedStateBlock))) != 0) {
            error = "ftruncate failed";
            close();
            return false;
        }
    } else {
        fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_ < 0) {
            error = "shm_open create failed";
            return false;
        }
        owner_ = true;
    }
    if (!mapInternal(fd_, error)) {
        close();
        return false;
    }
    const bool metadata_mismatch = (block_->magic != kSharedStateMagic || block_->version != kSharedStateVersion);
    if (created_new || metadata_mismatch) {
        std::memset(block_, 0, sizeof(SharedStateBlock));
        block_->magic = kSharedStateMagic;
        block_->version = kSharedStateVersion;
        block_->pause_map_updates.store(0U, std::memory_order_release);
        block_->shutdown_requested.store(0U, std::memory_order_release);
        block_->reload_generation.store(0U, std::memory_order_release);
        block_->log_level.store(0U, std::memory_order_release);
        block_->protocol_mismatch.store(0U, std::memory_order_release);
        for (uint32_t i = 0; i < static_cast<uint32_t>(RoleId::Count); ++i) {
            auto& r = block_->roles[i];
            r.heartbeat_ns.store(0U, std::memory_order_release);
            r.sequence_id.store(0U, std::memory_order_release);
            r.dropped.store(0U, std::memory_order_release);
            r.ready.store(0U, std::memory_order_release);
            std::memset(r.name, 0, sizeof(r.name));
            auto& s = block_->stage_kpi[i];
            s.samples.store(0U, std::memory_order_release);
            s.latency_last_ns.store(0U, std::memory_order_release);
            s.latency_mean_ns.store(0U, std::memory_order_release);
            s.latency_max_ns.store(0U, std::memory_order_release);
            s.latency_accum_ns.store(0U, std::memory_order_release);
            s.queue_depth.store(0U, std::memory_order_release);
            s.dropped.store(0U, std::memory_order_release);
        }
        block_->sync_kpi.sync_samples.store(0U, std::memory_order_release);
        block_->sync_kpi.interpolated_samples.store(0U, std::memory_order_release);
        block_->sync_kpi.stale_attitude_count.store(0U, std::memory_order_release);
        block_->sync_kpi.sync_error_abs_ns_accum.store(0U, std::memory_order_release);
        block_->sync_kpi.sync_error_abs_ns_max.store(0U, std::memory_order_release);
        block_->latest_camera.valid.store(0U, std::memory_order_release);
        block_->latest_fc.valid.store(0U, std::memory_order_release);
        block_->latest_bev.valid.store(0U, std::memory_order_release);
        block_->latest_map.valid.store(0U, std::memory_order_release);
        block_->thermal.valid.store(0U, std::memory_order_release);
        resetWatchdog(block_->watchdog);
    }
    return true;
#else
    (void)name;
    error = "SharedState requires Linux/POSIX shared memory";
    return false;
#endif
}

bool SharedState::attach(const std::string& name, std::string& error) {
    close();
#ifdef __linux__
    name_ = name;
    fd_ = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd_ < 0) {
        error = "shm_open attach failed";
        return false;
    }
    if (!mapInternal(fd_, error)) {
        close();
        return false;
    }
    if (block_->magic != kSharedStateMagic || block_->version != kSharedStateVersion) {
        error = "shared state metadata mismatch";
        close();
        return false;
    }
    return true;
#else
    (void)name;
    error = "SharedState requires Linux/POSIX shared memory";
    return false;
#endif
}

void SharedState::close() {
#ifdef __linux__
    if (block_) {
        munmap(block_, sizeof(SharedStateBlock));
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
#endif
    block_ = nullptr;
    fd_ = -1;
    owner_ = false;
    name_.clear();
}

void SharedState::setPauseMapUpdates(bool paused) {
    if (block_) {
        block_->pause_map_updates.store(paused ? 1U : 0U, std::memory_order_release);
    }
}

bool SharedState::pauseMapUpdates() const {
    return block_ && block_->pause_map_updates.load(std::memory_order_acquire) != 0U;
}

void SharedState::requestShutdown() {
    if (block_) {
        block_->shutdown_requested.store(1U, std::memory_order_release);
    }
}

bool SharedState::shutdownRequested() const {
    return block_ && block_->shutdown_requested.load(std::memory_order_acquire) != 0U;
}

uint32_t SharedState::reloadGeneration() const {
    return block_ ? block_->reload_generation.load(std::memory_order_acquire) : 0U;
}

uint32_t SharedState::requestReload() {
    if (!block_) {
        return 0U;
    }
    return block_->reload_generation.fetch_add(1U, std::memory_order_acq_rel) + 1U;
}

void SharedState::setProtocolMismatch(bool value) {
    if (block_) {
        block_->protocol_mismatch.store(value ? 1U : 0U, std::memory_order_release);
    }
}

bool SharedState::protocolMismatch() const {
    return block_ && block_->protocol_mismatch.load(std::memory_order_acquire) != 0U;
}

void SharedState::resetControlFlagsForStartup() {
    if (!block_) {
        return;
    }
    block_->pause_map_updates.store(0U, std::memory_order_release);
    block_->shutdown_requested.store(0U, std::memory_order_release);
    block_->protocol_mismatch.store(0U, std::memory_order_release);
}

void SharedState::resetRoleHealthForStartup() {
    if (!block_) {
        return;
    }
    for (uint32_t i = 0; i < static_cast<uint32_t>(RoleId::Count); ++i) {
        auto& r = block_->roles[i];
        r.heartbeat_ns.store(0U, std::memory_order_release);
        r.sequence_id.store(0U, std::memory_order_release);
        r.dropped.store(0U, std::memory_order_release);
        r.ready.store(0U, std::memory_order_release);
        std::memset(r.name, 0, sizeof(r.name));
        auto& s = block_->stage_kpi[i];
        s.samples.store(0U, std::memory_order_release);
        s.latency_last_ns.store(0U, std::memory_order_release);
        s.latency_mean_ns.store(0U, std::memory_order_release);
        s.latency_max_ns.store(0U, std::memory_order_release);
        s.latency_accum_ns.store(0U, std::memory_order_release);
        s.queue_depth.store(0U, std::memory_order_release);
        s.dropped.store(0U, std::memory_order_release);
    }
    block_->sync_kpi.sync_samples.store(0U, std::memory_order_release);
    block_->sync_kpi.interpolated_samples.store(0U, std::memory_order_release);
    block_->sync_kpi.stale_attitude_count.store(0U, std::memory_order_release);
    block_->sync_kpi.sync_error_abs_ns_accum.store(0U, std::memory_order_release);
    block_->sync_kpi.sync_error_abs_ns_max.store(0U, std::memory_order_release);
    block_->latest_camera.valid.store(0U, std::memory_order_release);
    block_->latest_fc.valid.store(0U, std::memory_order_release);
    block_->latest_bev.valid.store(0U, std::memory_order_release);
    block_->latest_map.valid.store(0U, std::memory_order_release);
    block_->thermal.valid.store(0U, std::memory_order_release);
    resetWatchdog(block_->watchdog);
}

void SharedState::recordStageKpi(RoleId role, uint64_t latency_ns, uint64_t queue_depth, uint64_t dropped) {
    if (!block_) {
        return;
    }
    auto& s = block_->stage_kpi[static_cast<uint32_t>(role)];
    const uint64_t n = s.samples.fetch_add(1U, std::memory_order_acq_rel) + 1U;
    s.latency_last_ns.store(latency_ns, std::memory_order_release);
    const uint64_t acc = s.latency_accum_ns.fetch_add(latency_ns, std::memory_order_acq_rel) + latency_ns;
    s.latency_mean_ns.store((n > 0U) ? (acc / n) : 0U, std::memory_order_release);
    uint64_t old_max = s.latency_max_ns.load(std::memory_order_acquire);
    while (latency_ns > old_max &&
           !s.latency_max_ns.compare_exchange_weak(
               old_max, latency_ns, std::memory_order_acq_rel, std::memory_order_acquire)) {
    }
    s.queue_depth.store(queue_depth, std::memory_order_release);
    s.dropped.store(dropped, std::memory_order_release);
}

void SharedState::publishLatestCamera(
    uint32_t frame_slot_id,
    uint32_t generation,
    uint32_t width,
    uint32_t height,
    uint32_t type,
    uint32_t stride,
    uint64_t sequence_id,
    int64_t timestamp_ns) {
    if (!block_) {
        return;
    }
    auto& s = block_->latest_camera;
    s.frame_slot_id.store(frame_slot_id, std::memory_order_release);
    s.generation.store(generation, std::memory_order_release);
    s.width.store(width, std::memory_order_release);
    s.height.store(height, std::memory_order_release);
    s.type.store(type, std::memory_order_release);
    s.stride.store(stride, std::memory_order_release);
    s.sequence_id.store(sequence_id, std::memory_order_release);
    s.timestamp_ns.store(timestamp_ns, std::memory_order_release);
    s.aux0.store(0U, std::memory_order_release);
    s.valid.store(1U, std::memory_order_release);
}

void SharedState::publishLatestFc(
    uint32_t sample_id,
    int64_t timestamp_ns,
    float roll_rad,
    float pitch_rad,
    float yaw_rad) {
    if (!block_) {
        return;
    }
    auto& s = block_->latest_fc;
    s.sample_id.store(sample_id, std::memory_order_release);
    s.timestamp_ns.store(timestamp_ns, std::memory_order_release);
    s.roll_rad_bits.store(packFloatBits(roll_rad), std::memory_order_release);
    s.pitch_rad_bits.store(packFloatBits(pitch_rad), std::memory_order_release);
    s.yaw_rad_bits.store(packFloatBits(yaw_rad), std::memory_order_release);
    s.valid.store(1U, std::memory_order_release);
}

void SharedState::publishLatestBev(
    uint32_t frame_slot_id,
    uint32_t generation,
    uint32_t width,
    uint32_t height,
    uint32_t type,
    uint32_t stride,
    uint64_t sequence_id,
    int64_t timestamp_ns,
    uint64_t aux0) {
    if (!block_) {
        return;
    }
    auto& s = block_->latest_bev;
    s.frame_slot_id.store(frame_slot_id, std::memory_order_release);
    s.generation.store(generation, std::memory_order_release);
    s.width.store(width, std::memory_order_release);
    s.height.store(height, std::memory_order_release);
    s.type.store(type, std::memory_order_release);
    s.stride.store(stride, std::memory_order_release);
    s.sequence_id.store(sequence_id, std::memory_order_release);
    s.timestamp_ns.store(timestamp_ns, std::memory_order_release);
    s.aux0.store(aux0, std::memory_order_release);
    s.valid.store(1U, std::memory_order_release);
}

void SharedState::publishLatestMap(
    uint32_t frame_slot_id,
    uint32_t generation,
    uint32_t width,
    uint32_t height,
    uint32_t type,
    uint32_t stride,
    uint64_t sequence_id,
    int64_t timestamp_ns,
    uint64_t aux0) {
    if (!block_) {
        return;
    }
    auto& s = block_->latest_map;
    s.frame_slot_id.store(frame_slot_id, std::memory_order_release);
    s.generation.store(generation, std::memory_order_release);
    s.width.store(width, std::memory_order_release);
    s.height.store(height, std::memory_order_release);
    s.type.store(type, std::memory_order_release);
    s.stride.store(stride, std::memory_order_release);
    s.sequence_id.store(sequence_id, std::memory_order_release);
    s.timestamp_ns.store(timestamp_ns, std::memory_order_release);
    s.aux0.store(aux0, std::memory_order_release);
    s.valid.store(1U, std::memory_order_release);
}

void SharedState::publishThermal(uint32_t temp_milli_c, bool throttled, uint32_t fps_cap) {
    if (!block_) {
        return;
    }
    auto& t = block_->thermal;
    t.temp_milli_c.store(temp_milli_c, std::memory_order_release);
    t.throttled.store(throttled ? 1U : 0U, std::memory_order_release);
    t.fps_cap.store(fps_cap, std::memory_order_release);
    t.valid.store(1U, std::memory_order_release);
}

void SharedState::addSyncSample(bool interpolated, bool stale, uint64_t abs_error_ns) {
    if (!block_) {
        return;
    }
    auto& k = block_->sync_kpi;
    k.sync_samples.fetch_add(1U, std::memory_order_acq_rel);
    if (interpolated) {
        k.interpolated_samples.fetch_add(1U, std::memory_order_acq_rel);
    }
    if (stale) {
        k.stale_attitude_count.fetch_add(1U, std::memory_order_acq_rel);
        auto& w = block_->watchdog;
        w.msp_stale_total.fetch_add(1U, std::memory_order_acq_rel);
        const uint64_t streak = w.msp_stale_streak.fetch_add(1U, std::memory_order_acq_rel) + 1U;
        if (streak >= 10U) {
            w.alert_msp_stale.store(1U, std::memory_order_release);
        }
    } else {
        block_->watchdog.msp_stale_streak.store(0U, std::memory_order_release);
        block_->watchdog.alert_msp_stale.store(0U, std::memory_order_release);
    }
    k.sync_error_abs_ns_accum.fetch_add(abs_error_ns, std::memory_order_acq_rel);

    uint64_t old_max = k.sync_error_abs_ns_max.load(std::memory_order_acquire);
    while (abs_error_ns > old_max &&
           !k.sync_error_abs_ns_max.compare_exchange_weak(
               old_max, abs_error_ns, std::memory_order_acq_rel, std::memory_order_acquire)) {
    }
}

void SharedState::publishTrackerWatchdog(uint64_t failure_streak, bool reinitialized) {
    if (!block_) {
        return;
    }
    auto& w = block_->watchdog;
    w.tracker_failure_streak.store(failure_streak, std::memory_order_release);
    if (reinitialized) {
        w.tracker_reinit_total.fetch_add(1U, std::memory_order_acq_rel);
    }
    w.alert_tracker_failure.store((failure_streak >= 5U) ? 1U : 0U, std::memory_order_release);
}

void SharedState::publishMapWatchdog(uint64_t pause_streak, bool unstable_event) {
    if (!block_) {
        return;
    }
    auto& w = block_->watchdog;
    w.map_pause_streak.store(pause_streak, std::memory_order_release);
    if (unstable_event) {
        w.map_instability_total.fetch_add(1U, std::memory_order_acq_rel);
    }
    w.alert_map_instability.store((pause_streak >= 30U) ? 1U : 0U, std::memory_order_release);
}

SyncKpiSnapshot SharedState::syncKpiSnapshot() const {
    SyncKpiSnapshot out{};
    if (!block_) {
        return out;
    }
    const auto& k = block_->sync_kpi;
    out.sync_samples = k.sync_samples.load(std::memory_order_acquire);
    out.interpolated_samples = k.interpolated_samples.load(std::memory_order_acquire);
    out.stale_attitude_count = k.stale_attitude_count.load(std::memory_order_acquire);
    out.sync_error_abs_ns_accum = k.sync_error_abs_ns_accum.load(std::memory_order_acquire);
    out.sync_error_abs_ns_max = k.sync_error_abs_ns_max.load(std::memory_order_acquire);
    return out;
}

void SharedState::markReady(RoleId role, const char* name) {
    if (!block_) {
        return;
    }
    auto& r = block_->roles[static_cast<uint32_t>(role)];
    r.ready.store(1U, std::memory_order_release);
    std::memset(r.name, 0, sizeof(r.name));
    if (name) {
        std::strncpy(r.name, name, sizeof(r.name) - 1);
    }
}

void SharedState::updateHeartbeat(RoleId role, uint64_t now_ns, uint64_t sequence_id, uint64_t dropped) {
    if (!block_) {
        return;
    }
    auto& r = block_->roles[static_cast<uint32_t>(role)];
    r.ready.store(1U, std::memory_order_release);
    r.heartbeat_ns.store(now_ns, std::memory_order_release);
    r.sequence_id.store(sequence_id, std::memory_order_release);
    r.dropped.store(dropped, std::memory_order_release);
}

}  // namespace bev::ipc
