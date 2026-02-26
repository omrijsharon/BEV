#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/types.hpp"

namespace bev {

class TrackManager {
public:
    struct RecoveryState {
        bool map_updates_paused{false};
        bool should_reinitialize{false};
        int low_confidence_streak{0};
        int stable_streak{0};
    };

    explicit TrackManager(float max_predict_distance_px);

    cv::Point2f predictSearchCenter(const TrackPoint& track, float dt_s) const;
    static cv::Point2f clampDelta(const cv::Point2f& delta, float max_magnitude);
    static int countValidTracks(const std::vector<TrackPoint>& tracks);
    static bool shouldReinitialize(const std::vector<TrackPoint>& tracks, int min_valid_tracks);
    static void invalidateAll(std::vector<TrackPoint>& tracks);
    static float meanConfidence(const std::vector<TrackPoint>& tracks);
    static RecoveryState updateRecoveryState(
        const RecoveryState& prev,
        const std::vector<TrackPoint>& tracks,
        int min_valid_tracks,
        float min_confidence,
        int stable_frames_to_resume);

private:
    float max_predict_distance_px_{30.0F};
};

}  // namespace bev
