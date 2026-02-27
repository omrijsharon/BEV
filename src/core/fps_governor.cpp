#include "core/fps_governor.hpp"

#include <algorithm>

namespace bev {

AdaptiveFpsGovernor::AdaptiveFpsGovernor(const IpcConfig& cfg) {
    updateConfig(cfg);
    reset();
}

void AdaptiveFpsGovernor::updateConfig(const IpcConfig& cfg) {
    enabled_ = cfg.adaptive_camera_fps_enable;
    fps_min_ = std::max(1, cfg.adaptive_camera_fps_min);
    fps_max_ = std::max(fps_min_, cfg.adaptive_camera_fps_max);
    fps_step_ = std::max(1, cfg.adaptive_camera_fps_step);
    backlog_low_ = std::max(0, cfg.adaptive_backlog_low);
    backlog_high_ = std::max(backlog_low_, cfg.adaptive_backlog_high);
    stable_up_ = std::max(1, cfg.adaptive_stable_intervals_for_up);
    stable_down_ = std::max(1, cfg.adaptive_stable_intervals_for_down);
    current_fps_ = std::clamp(current_fps_, fps_min_, fps_max_);
}

void AdaptiveFpsGovernor::reset() {
    low_streak_ = 0;
    high_streak_ = 0;
    current_fps_ = std::clamp(current_fps_, fps_min_, fps_max_);
}

int AdaptiveFpsGovernor::update(uint64_t backlog_depth, uint64_t dropped_delta) {
    if (!enabled_) {
        return current_fps_;
    }
    const bool overloaded = (backlog_depth >= static_cast<uint64_t>(backlog_high_)) || (dropped_delta > 0U);
    const bool underloaded = (backlog_depth <= static_cast<uint64_t>(backlog_low_)) && (dropped_delta == 0U);

    if (overloaded) {
        high_streak_ += 1;
        low_streak_ = 0;
        if (high_streak_ >= stable_down_) {
            current_fps_ = std::max(fps_min_, current_fps_ - fps_step_);
            high_streak_ = 0;
        }
        return current_fps_;
    }

    if (underloaded) {
        low_streak_ += 1;
        high_streak_ = 0;
        if (low_streak_ >= stable_up_) {
            current_fps_ = std::min(fps_max_, current_fps_ + fps_step_);
            low_streak_ = 0;
        }
        return current_fps_;
    }

    low_streak_ = 0;
    high_streak_ = 0;
    return current_fps_;
}

}  // namespace bev

