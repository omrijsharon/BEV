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

float TrackManager::estimateOmegaRadPerSec(
    const std::vector<cv::Point2f>& prev_points,
    const std::vector<cv::Point2f>& curr_points,
    float dt_s,
    const std::vector<float>* weights) {
    if (dt_s <= 1e-6F ||
        prev_points.size() < 2 ||
        prev_points.size() != curr_points.size() ||
        (weights != nullptr && weights->size() != prev_points.size())) {
        return 0.0F;
    }

    cv::Point2f c_prev(0.0F, 0.0F);
    cv::Point2f c_curr(0.0F, 0.0F);
    float w_sum = 0.0F;
    for (std::size_t i = 0; i < prev_points.size(); ++i) {
        const float w = (weights == nullptr) ? 1.0F : std::max(0.0F, (*weights)[i]);
        c_prev += prev_points[i] * w;
        c_curr += curr_points[i] * w;
        w_sum += w;
    }
    if (w_sum <= 1e-6F) {
        return 0.0F;
    }
    c_prev *= (1.0F / w_sum);
    c_curr *= (1.0F / w_sum);

    double num = 0.0;
    double den = 0.0;
    for (std::size_t i = 0; i < prev_points.size(); ++i) {
        const float w = (weights == nullptr) ? 1.0F : std::max(0.0F, (*weights)[i]);
        const cv::Point2f rp = prev_points[i] - c_prev;
        const cv::Point2f rc = curr_points[i] - c_curr;
        num += static_cast<double>(w) * static_cast<double>(rp.x * rc.y - rp.y * rc.x);
        den += static_cast<double>(w) * static_cast<double>(rp.x * rc.x + rp.y * rc.y);
    }
    if (std::abs(num) < 1e-12 && std::abs(den) < 1e-12) {
        return 0.0F;
    }
    const double theta = std::atan2(num, den);
    return static_cast<float>(theta / static_cast<double>(dt_s));
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
