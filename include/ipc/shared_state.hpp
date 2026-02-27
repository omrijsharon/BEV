#pragma once

#include <atomic>
#include <cstdint>
#include <string>

namespace bev::ipc {

enum class RoleId : uint32_t {
    Camera = 0,
    Fc = 1,
    Bev = 2,
    Track = 3,
    Map = 4,
    Web = 5,
    Count = 6,
};

struct alignas(64) RoleHealth {
    std::atomic<uint64_t> heartbeat_ns;
    std::atomic<uint64_t> sequence_id;
    std::atomic<uint64_t> dropped;
    std::atomic<uint32_t> ready;
    char name[16];
    uint8_t reserved[28];
};

struct alignas(64) SyncKpi {
    std::atomic<uint64_t> sync_samples;
    std::atomic<uint64_t> interpolated_samples;
    std::atomic<uint64_t> stale_attitude_count;
    std::atomic<uint64_t> sync_error_abs_ns_accum;
    std::atomic<uint64_t> sync_error_abs_ns_max;
    uint8_t reserved[24];
};

struct SyncKpiSnapshot {
    uint64_t sync_samples{0};
    uint64_t interpolated_samples{0};
    uint64_t stale_attitude_count{0};
    uint64_t sync_error_abs_ns_accum{0};
    uint64_t sync_error_abs_ns_max{0};
};

struct alignas(64) WatchdogSnapshot {
    std::atomic<uint64_t> msp_stale_total;
    std::atomic<uint64_t> msp_stale_streak;
    std::atomic<uint64_t> tracker_reinit_total;
    std::atomic<uint64_t> tracker_failure_streak;
    std::atomic<uint64_t> map_pause_streak;
    std::atomic<uint64_t> map_instability_total;
    std::atomic<uint32_t> alert_msp_stale;
    std::atomic<uint32_t> alert_tracker_failure;
    std::atomic<uint32_t> alert_map_instability;
    uint8_t reserved[36];
};

struct alignas(64) StageKpi {
    std::atomic<uint64_t> samples;
    std::atomic<uint64_t> latency_last_ns;
    std::atomic<uint64_t> latency_mean_ns;
    std::atomic<uint64_t> latency_max_ns;
    std::atomic<uint64_t> latency_accum_ns;
    std::atomic<uint64_t> queue_depth;
    std::atomic<uint64_t> dropped;
    uint8_t reserved[8];
};

struct alignas(64) LatestFrameSnapshot {
    std::atomic<uint32_t> valid;
    std::atomic<uint32_t> frame_slot_id;
    std::atomic<uint32_t> generation;
    std::atomic<uint32_t> width;
    std::atomic<uint32_t> height;
    std::atomic<uint32_t> type;
    std::atomic<uint32_t> stride;
    std::atomic<uint64_t> sequence_id;
    std::atomic<int64_t> timestamp_ns;
    std::atomic<uint64_t> aux0;
    uint8_t reserved[8];
};

struct alignas(64) LatestAttitudeSnapshot {
    std::atomic<uint32_t> valid;
    std::atomic<uint32_t> sample_id;
    std::atomic<int64_t> timestamp_ns;
    std::atomic<uint32_t> roll_rad_bits;
    std::atomic<uint32_t> pitch_rad_bits;
    std::atomic<uint32_t> yaw_rad_bits;
    uint8_t reserved[32];
};

struct alignas(64) ThermalSnapshot {
    std::atomic<uint32_t> valid;
    std::atomic<uint32_t> throttled;
    std::atomic<uint32_t> temp_milli_c;
    std::atomic<uint32_t> fps_cap;
    uint8_t reserved[48];
};

struct alignas(64) SharedStateBlock {
    uint32_t magic;
    uint32_t version;
    std::atomic<uint32_t> pause_map_updates;
    std::atomic<uint32_t> shutdown_requested;
    std::atomic<uint32_t> reload_generation;
    std::atomic<uint32_t> log_level;
    std::atomic<uint32_t> protocol_mismatch;
    SyncKpi sync_kpi;
    RoleHealth roles[static_cast<uint32_t>(RoleId::Count)];
    StageKpi stage_kpi[static_cast<uint32_t>(RoleId::Count)];
    LatestFrameSnapshot latest_camera;
    LatestAttitudeSnapshot latest_fc;
    LatestFrameSnapshot latest_bev;
    LatestFrameSnapshot latest_map;
    ThermalSnapshot thermal;
    WatchdogSnapshot watchdog;
};

class SharedState {
public:
    SharedState() = default;
    ~SharedState();

    bool create(const std::string& name, std::string& error);
    bool attach(const std::string& name, std::string& error);
    void close();

    bool isOpen() const { return block_ != nullptr; }

    void setPauseMapUpdates(bool paused);
    bool pauseMapUpdates() const;

    void requestShutdown();
    bool shutdownRequested() const;
    uint32_t reloadGeneration() const;
    uint32_t requestReload();
    void setProtocolMismatch(bool value);
    bool protocolMismatch() const;
    void resetControlFlagsForStartup();
    void resetRoleHealthForStartup();
    void recordStageKpi(RoleId role, uint64_t latency_ns, uint64_t queue_depth, uint64_t dropped);
    void publishLatestCamera(uint32_t frame_slot_id, uint32_t generation, uint32_t width, uint32_t height, uint32_t type, uint32_t stride, uint64_t sequence_id, int64_t timestamp_ns);
    void publishLatestFc(uint32_t sample_id, int64_t timestamp_ns, float roll_rad, float pitch_rad, float yaw_rad);
    void publishLatestBev(uint32_t frame_slot_id, uint32_t generation, uint32_t width, uint32_t height, uint32_t type, uint32_t stride, uint64_t sequence_id, int64_t timestamp_ns, uint64_t aux0);
    void publishLatestMap(uint32_t frame_slot_id, uint32_t generation, uint32_t width, uint32_t height, uint32_t type, uint32_t stride, uint64_t sequence_id, int64_t timestamp_ns, uint64_t aux0);
    void publishThermal(uint32_t temp_milli_c, bool throttled, uint32_t fps_cap);
    void addSyncSample(bool interpolated, bool stale, uint64_t abs_error_ns);
    void publishTrackerWatchdog(uint64_t failure_streak, bool reinitialized);
    void publishMapWatchdog(uint64_t pause_streak, bool unstable_event);
    SyncKpiSnapshot syncKpiSnapshot() const;

    void markReady(RoleId role, const char* name);
    void updateHeartbeat(RoleId role, uint64_t now_ns, uint64_t sequence_id, uint64_t dropped);
    const SharedStateBlock* block() const { return block_; }

private:
    bool mapInternal(int fd, std::string& error);

    std::string name_{};
    int fd_{-1};
    bool owner_{false};
    SharedStateBlock* block_{nullptr};
};

}  // namespace bev::ipc
