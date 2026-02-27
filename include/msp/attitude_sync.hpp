#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "ipc/descriptors.hpp"

namespace bev {

struct SyncPolicy {
    int64_t max_attitude_age_ns{100000000LL};
    int64_t max_interp_gap_ns{60000000LL};
    std::size_t startup_min_buffer{4U};
};

struct SyncResult {
    bool ok{false};
    bool interpolated{false};
    int64_t sync_error_ns{0};
    bev::ipc::AttitudeDesc attitude{};
};

class AttitudeSyncBuffer {
public:
    explicit AttitudeSyncBuffer(std::size_t max_samples = 2048U);

    void push(const bev::ipc::AttitudeDesc& sample);
    std::size_t size() const { return samples_.size(); }
    SyncResult resolve(int64_t frame_timestamp_ns, const SyncPolicy& policy) const;

private:
    static bev::ipc::AttitudeDesc lerpEuler(const bev::ipc::AttitudeDesc& a, const bev::ipc::AttitudeDesc& b, int64_t ts);

    std::size_t max_samples_;
    std::deque<bev::ipc::AttitudeDesc> samples_;
};

}  // namespace bev
