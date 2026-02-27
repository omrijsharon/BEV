#include "core/fps_governor.hpp"

#include <iostream>

int main() {
    bev::IpcConfig cfg;
    cfg.adaptive_camera_fps_enable = true;
    cfg.adaptive_camera_fps_min = 8;
    cfg.adaptive_camera_fps_max = 20;
    cfg.adaptive_camera_fps_step = 2;
    cfg.adaptive_backlog_low = 1;
    cfg.adaptive_backlog_high = 6;
    cfg.adaptive_stable_intervals_for_up = 2;
    cfg.adaptive_stable_intervals_for_down = 1;

    bev::AdaptiveFpsGovernor g(cfg);

    // Overload should quickly reduce FPS.
    int fps = 15;
    g.updateConfig(cfg);
    for (int i = 0; i < 3; ++i) {
        fps = g.update(10, 0);
    }
    if (fps >= 15) {
        std::cerr << "fps should reduce under overload\n";
        return 1;
    }
    if (fps < cfg.adaptive_camera_fps_min) {
        std::cerr << "fps should not drop below min\n";
        return 1;
    }

    // Idle low backlog should increase FPS toward max with hysteresis.
    for (int i = 0; i < 20; ++i) {
        fps = g.update(0, 0);
    }
    if (fps > cfg.adaptive_camera_fps_max) {
        std::cerr << "fps should not exceed max\n";
        return 1;
    }
    if (fps < 12) {
        std::cerr << "fps should recover when underloaded\n";
        return 1;
    }

    // Drops should trigger downshift.
    const int before_drop = fps;
    fps = g.update(0, 3);
    if (fps >= before_drop) {
        std::cerr << "drop delta should reduce fps\n";
        return 1;
    }

    return 0;
}

