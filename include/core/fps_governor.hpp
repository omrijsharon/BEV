#pragma once

#include "core/config.hpp"

#include <cstdint>

namespace bev {

class AdaptiveFpsGovernor {
public:
    explicit AdaptiveFpsGovernor(const IpcConfig& cfg);

    void reset();
    void updateConfig(const IpcConfig& cfg);
    int currentFps() const { return current_fps_; }
    int update(uint64_t backlog_depth, uint64_t dropped_delta);

private:
    bool enabled_{true};
    int fps_min_{8};
    int fps_max_{30};
    int fps_step_{1};
    int backlog_low_{1};
    int backlog_high_{8};
    int stable_up_{2};
    int stable_down_{1};
    int current_fps_{15};
    int low_streak_{0};
    int high_streak_{0};
};

}  // namespace bev

