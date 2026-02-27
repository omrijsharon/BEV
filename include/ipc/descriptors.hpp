#pragma once

#include <cstdint>
#include <type_traits>

namespace bev::ipc {

constexpr uint32_t kIpcProtocolVersion = 1;

#pragma pack(push, 1)
struct BevFrameDesc {
    uint32_t protocol_version;
    uint32_t frame_slot_id;
    uint32_t generation;
    uint32_t width;
    uint32_t height;
    uint32_t type;
    uint32_t stride;
    uint64_t sequence_id;
    int64_t timestamp_ns;
    uint64_t reserved0;
};

struct FrameDesc {
    uint32_t protocol_version;
    uint32_t frame_slot_id;
    uint32_t generation;
    uint32_t width;
    uint32_t height;
    uint32_t type;
    uint32_t stride;
    uint64_t sequence_id;
    int64_t timestamp_ns;
    uint64_t reserved0;
};

struct AttitudeDesc {
    uint32_t protocol_version;
    uint32_t sample_id;
    int64_t timestamp_ns;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float qx;
    float qy;
    float qz;
    float qw;
    uint64_t reserved0;
};

struct TrackResultDesc {
    uint32_t protocol_version;
    uint32_t frame_slot_id;
    uint32_t generation;
    uint32_t valid_track_count;
    uint64_t sequence_id;
    int64_t timestamp_ns;
    float vx_px_s;
    float vy_px_s;
    float omega_rad_s;
    float inlier_ratio;
    float confidence;
    float reproj_rmse;
    uint64_t reserved0;
};

struct MapFrameDesc {
    uint32_t protocol_version;
    uint32_t frame_slot_id;
    uint32_t generation;
    uint32_t map_width;
    uint32_t map_height;
    uint64_t sequence_id;
    int64_t timestamp_ns;
    float coverage_ratio;
    float drift_px;
    float map_confidence;
    uint8_t map_paused;
    uint8_t reserved[7];
    uint64_t reserved0;
};
#pragma pack(pop)

static_assert(std::is_trivially_copyable<BevFrameDesc>::value, "BevFrameDesc must be trivially copyable");
static_assert(std::is_trivially_copyable<FrameDesc>::value, "FrameDesc must be trivially copyable");
static_assert(std::is_trivially_copyable<AttitudeDesc>::value, "AttitudeDesc must be trivially copyable");
static_assert(std::is_trivially_copyable<TrackResultDesc>::value, "TrackResultDesc must be trivially copyable");
static_assert(std::is_trivially_copyable<MapFrameDesc>::value, "MapFrameDesc must be trivially copyable");

static_assert(sizeof(BevFrameDesc) == 52, "BevFrameDesc ABI size changed");
static_assert(sizeof(FrameDesc) == 52, "FrameDesc ABI size changed");
static_assert(sizeof(AttitudeDesc) == 52, "AttitudeDesc ABI size changed");
static_assert(sizeof(TrackResultDesc) == 64, "TrackResultDesc ABI size changed");
static_assert(sizeof(MapFrameDesc) == 64, "MapFrameDesc ABI size changed");

inline BevFrameDesc makeBevFrameDesc() {
    BevFrameDesc d{};
    d.protocol_version = kIpcProtocolVersion;
    return d;
}

inline TrackResultDesc makeTrackResultDesc() {
    TrackResultDesc d{};
    d.protocol_version = kIpcProtocolVersion;
    return d;
}

inline FrameDesc makeFrameDesc() {
    FrameDesc d{};
    d.protocol_version = kIpcProtocolVersion;
    return d;
}

inline AttitudeDesc makeAttitudeDesc() {
    AttitudeDesc d{};
    d.protocol_version = kIpcProtocolVersion;
    return d;
}

inline MapFrameDesc makeMapFrameDesc() {
    MapFrameDesc d{};
    d.protocol_version = kIpcProtocolVersion;
    return d;
}

}  // namespace bev::ipc
