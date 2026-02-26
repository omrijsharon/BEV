#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include "core/types.hpp"

namespace bev {

class MSPBridge {
public:
    explicit MSPBridge(std::size_t max_samples = 512U);

    void pushAttitudeSample(const AttitudeSample& sample);
    std::optional<AttitudeSample> getNearestSample(int64_t timestamp_ns) const;
    std::optional<AttitudeSample> getInterpolatedSample(int64_t timestamp_ns) const;

    std::size_t size() const;

private:
    std::size_t max_samples_;
    mutable std::mutex mutex_;
    std::deque<AttitudeSample> samples_;
};

}  // namespace bev
