#pragma once

#include <atomic>

namespace bev {

struct TelemetrySnapshot {
    int fps{0};
    int valid_tracks{0};
    int inliers{0};
    int frame_latency_ms{0};
};

class Telemetry {
public:
    void setFps(int value);
    void setValidTracks(int value);
    void setInliers(int value);
    void setFrameLatencyMs(int value);

    TelemetrySnapshot snapshot() const;

private:
    std::atomic<int> fps_{0};
    std::atomic<int> valid_tracks_{0};
    std::atomic<int> inliers_{0};
    std::atomic<int> frame_latency_ms_{0};
};

}  // namespace bev