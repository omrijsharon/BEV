#include "core/telemetry.hpp"

namespace bev {

void Telemetry::setFps(int value) { fps_.store(value); }
void Telemetry::setValidTracks(int value) { valid_tracks_.store(value); }
void Telemetry::setInliers(int value) { inliers_.store(value); }
void Telemetry::setFrameLatencyMs(int value) { frame_latency_ms_.store(value); }

TelemetrySnapshot Telemetry::snapshot() const {
    return TelemetrySnapshot{
        fps_.load(),
        valid_tracks_.load(),
        inliers_.load(),
        frame_latency_ms_.load()};
}

}  // namespace bev