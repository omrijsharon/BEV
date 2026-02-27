#include "msp/attitude_sync.hpp"

#include <algorithm>
#include <cmath>

namespace bev {

namespace {

float lerpf(float a, float b, float t) {
    return a + (b - a) * t;
}

}  // namespace

AttitudeSyncBuffer::AttitudeSyncBuffer(std::size_t max_samples)
    : max_samples_(max_samples) {}

void AttitudeSyncBuffer::push(const bev::ipc::AttitudeDesc& sample) {
    if (!samples_.empty() && sample.timestamp_ns < samples_.back().timestamp_ns) {
        const auto it = std::lower_bound(
            samples_.begin(), samples_.end(), sample.timestamp_ns,
            [](const bev::ipc::AttitudeDesc& lhs, int64_t rhs_ts) { return lhs.timestamp_ns < rhs_ts; });
        samples_.insert(it, sample);
    } else {
        samples_.push_back(sample);
    }

    while (samples_.size() > max_samples_) {
        samples_.pop_front();
    }
}

bev::ipc::AttitudeDesc AttitudeSyncBuffer::lerpEuler(
    const bev::ipc::AttitudeDesc& a, const bev::ipc::AttitudeDesc& b, int64_t ts) {
    bev::ipc::AttitudeDesc out = a;
    out.timestamp_ns = ts;
    const double denom = static_cast<double>(b.timestamp_ns - a.timestamp_ns);
    const float t = (denom <= 0.0)
        ? 0.0F
        : static_cast<float>(static_cast<double>(ts - a.timestamp_ns) / denom);
    out.roll_rad = lerpf(a.roll_rad, b.roll_rad, t);
    out.pitch_rad = lerpf(a.pitch_rad, b.pitch_rad, t);
    out.yaw_rad = lerpf(a.yaw_rad, b.yaw_rad, t);
    out.qx = lerpf(a.qx, b.qx, t);
    out.qy = lerpf(a.qy, b.qy, t);
    out.qz = lerpf(a.qz, b.qz, t);
    out.qw = lerpf(a.qw, b.qw, t);
    return out;
}

SyncResult AttitudeSyncBuffer::resolve(int64_t frame_timestamp_ns, const SyncPolicy& policy) const {
    SyncResult out{};
    if (samples_.size() < policy.startup_min_buffer || samples_.empty()) {
        return out;
    }

    const auto it = std::lower_bound(
        samples_.begin(), samples_.end(), frame_timestamp_ns,
        [](const bev::ipc::AttitudeDesc& lhs, int64_t rhs_ts) { return lhs.timestamp_ns < rhs_ts; });

    if (it != samples_.begin() && it != samples_.end()) {
        const auto& b = *it;
        const auto& a = *(it - 1);
        const int64_t gap = b.timestamp_ns - a.timestamp_ns;
        if (gap > 0 && gap <= policy.max_interp_gap_ns) {
            out.ok = true;
            out.interpolated = true;
            out.attitude = lerpEuler(a, b, frame_timestamp_ns);
            out.sync_error_ns = 0;
            return out;
        }
    }

    bev::ipc::AttitudeDesc nearest{};
    if (it == samples_.begin()) {
        nearest = *it;
    } else if (it == samples_.end()) {
        nearest = samples_.back();
    } else {
        const auto& right = *it;
        const auto& left = *(it - 1);
        const int64_t dl = std::llabs(frame_timestamp_ns - left.timestamp_ns);
        const int64_t dr = std::llabs(right.timestamp_ns - frame_timestamp_ns);
        nearest = (dr < dl) ? right : left;
    }

    const int64_t age = std::llabs(frame_timestamp_ns - nearest.timestamp_ns);
    if (age > policy.max_attitude_age_ns) {
        return out;
    }
    out.ok = true;
    out.interpolated = false;
    out.attitude = nearest;
    out.sync_error_ns = frame_timestamp_ns - nearest.timestamp_ns;
    return out;
}

}  // namespace bev
