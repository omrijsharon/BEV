#include "tracking/track_manager.hpp"

#include <cmath>

namespace bev {

TrackManager::TrackManager(float max_predict_distance_px)
    : max_predict_distance_px_(max_predict_distance_px) {}

cv::Point2f TrackManager::clampDelta(const cv::Point2f& delta, float max_magnitude) {
    const float magnitude = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (magnitude <= max_magnitude || magnitude <= 1e-6F) {
        return delta;
    }
    const float scale = max_magnitude / magnitude;
    return cv::Point2f(delta.x * scale, delta.y * scale);
}

cv::Point2f TrackManager::predictSearchCenter(const TrackPoint& track, float dt_s) const {
    const cv::Point2f predicted_delta = track.vel_px_s * dt_s;
    const cv::Point2f clamped = clampDelta(predicted_delta, max_predict_distance_px_);
    return track.pos + clamped;
}

int TrackManager::countValidTracks(const std::vector<TrackPoint>& tracks) {
    int count = 0;
    for (const auto& t : tracks) {
        if (t.valid) {
            ++count;
        }
    }
    return count;
}

bool TrackManager::shouldReinitialize(const std::vector<TrackPoint>& tracks, int min_valid_tracks) {
    return countValidTracks(tracks) < min_valid_tracks;
}

void TrackManager::invalidateAll(std::vector<TrackPoint>& tracks) {
    for (auto& t : tracks) {
        t.valid = false;
        t.confidence = 0.0F;
    }
}

float TrackManager::meanConfidence(const std::vector<TrackPoint>& tracks) {
    float sum = 0.0F;
    int count = 0;
    for (const auto& t : tracks) {
        if (!t.valid) {
            continue;
        }
        sum += t.confidence;
        ++count;
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0F;
}

TrackManager::RecoveryState TrackManager::updateRecoveryState(
    const RecoveryState& prev,
    const std::vector<TrackPoint>& tracks,
    int min_valid_tracks,
    float min_confidence,
    int stable_frames_to_resume) {
    RecoveryState out = prev;
    out.should_reinitialize = false;

    const int valid = countValidTracks(tracks);
    const float conf = meanConfidence(tracks);
    const bool weak = (valid < min_valid_tracks) || (conf < min_confidence);

    if (weak) {
        out.low_confidence_streak += 1;
        out.stable_streak = 0;
        out.map_updates_paused = true;
        out.should_reinitialize = true;
        return out;
    }

    out.low_confidence_streak = 0;
    out.stable_streak += 1;
    if (out.stable_streak >= stable_frames_to_resume) {
        out.map_updates_paused = false;
    }
    return out;
}

}  // namespace bev
