#include "msp/msp_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace bev {

namespace {

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

}  // namespace

MSPBridge::MSPBridge(std::size_t max_samples)
    : max_samples_(max_samples) {}

void MSPBridge::pushAttitudeSample(const AttitudeSample& sample) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!samples_.empty() && sample.timestamp_ns < samples_.back().timestamp_ns) {
        const auto it = std::lower_bound(
            samples_.begin(), samples_.end(), sample.timestamp_ns,
            [](const AttitudeSample& lhs, int64_t rhs_timestamp) {
                return lhs.timestamp_ns < rhs_timestamp;
            });
        samples_.insert(it, sample);
    } else {
        samples_.push_back(sample);
    }

    while (samples_.size() > max_samples_) {
        samples_.pop_front();
    }
}

std::optional<AttitudeSample> MSPBridge::getNearestSample(int64_t timestamp_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {
        return std::nullopt;
    }

    const auto it = std::lower_bound(
        samples_.begin(), samples_.end(), timestamp_ns,
        [](const AttitudeSample& lhs, int64_t rhs_timestamp) {
            return lhs.timestamp_ns < rhs_timestamp;
        });

    if (it == samples_.begin()) {
        return *it;
    }
    if (it == samples_.end()) {
        return samples_.back();
    }

    const AttitudeSample& right = *it;
    const AttitudeSample& left = *(it - 1);
    const int64_t left_delta = std::llabs(timestamp_ns - left.timestamp_ns);
    const int64_t right_delta = std::llabs(right.timestamp_ns - timestamp_ns);
    return (right_delta < left_delta) ? right : left;
}

std::optional<AttitudeSample> MSPBridge::getInterpolatedSample(int64_t timestamp_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {
        return std::nullopt;
    }

    const auto it = std::lower_bound(
        samples_.begin(), samples_.end(), timestamp_ns,
        [](const AttitudeSample& lhs, int64_t rhs_timestamp) {
            return lhs.timestamp_ns < rhs_timestamp;
        });

    if (it == samples_.begin()) {
        return *it;
    }
    if (it == samples_.end()) {
        return samples_.back();
    }
    if (it->timestamp_ns == timestamp_ns) {
        return *it;
    }

    const AttitudeSample& right = *it;
    const AttitudeSample& left = *(it - 1);

    const double dt = static_cast<double>(right.timestamp_ns - left.timestamp_ns);
    if (dt <= 0.0) {
        return left;
    }
    const double t = static_cast<double>(timestamp_ns - left.timestamp_ns) / dt;

    AttitudeSample out;
    out.timestamp_ns = timestamp_ns;
    out.roll_rad = lerp(left.roll_rad, right.roll_rad, t);
    out.pitch_rad = lerp(left.pitch_rad, right.pitch_rad, t);
    out.yaw_rad = lerp(left.yaw_rad, right.yaw_rad, t);
    return out;
}

std::size_t MSPBridge::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

}  // namespace bev
