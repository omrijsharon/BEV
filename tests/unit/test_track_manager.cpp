#include "tracking/track_manager.hpp"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    bev::TrackManager manager(10.0F);

    bev::TrackPoint track;
    track.pos = cv::Point2f(100.0F, 100.0F);
    track.vel_px_s = cv::Point2f(8.0F, 6.0F);

    const cv::Point2f predicted = manager.predictSearchCenter(track, 1.0F);
    const float dx = predicted.x - track.pos.x;
    const float dy = predicted.y - track.pos.y;
    const float magnitude = std::sqrt(dx * dx + dy * dy);

    if (magnitude > 10.001F) {
        std::cerr << "predicted delta should be clamped\n";
        return 1;
    }

    std::vector<bev::TrackPoint> tracks(3);
    tracks[0].valid = true;
    tracks[1].valid = false;
    tracks[2].valid = true;
    if (bev::TrackManager::countValidTracks(tracks) != 2) {
        std::cerr << "valid track count mismatch\n";
        return 1;
    }
    if (!bev::TrackManager::shouldReinitialize(tracks, 3)) {
        std::cerr << "reinitialize should be required\n";
        return 1;
    }

    bev::TrackManager::invalidateAll(tracks);
    if (bev::TrackManager::countValidTracks(tracks) != 0) {
        std::cerr << "invalidateAll failed\n";
        return 1;
    }

    std::vector<bev::TrackPoint> rec_tracks(4);
    for (auto& t : rec_tracks) {
        t.valid = true;
        t.confidence = 0.9F;
    }
    bev::TrackManager::RecoveryState state{};
    state = bev::TrackManager::updateRecoveryState(state, rec_tracks, 3, 0.5F, 2);
    if (state.map_updates_paused) {
        std::cerr << "should stay running on good confidence\n";
        return 1;
    }

    rec_tracks[0].valid = false;
    rec_tracks[1].valid = false;
    state = bev::TrackManager::updateRecoveryState(state, rec_tracks, 3, 0.5F, 2);
    if (!state.map_updates_paused || !state.should_reinitialize) {
        std::cerr << "should pause and request reinit on weak tracks\n";
        return 1;
    }

    for (auto& t : rec_tracks) {
        t.valid = true;
        t.confidence = 0.9F;
    }
    state = bev::TrackManager::updateRecoveryState(state, rec_tracks, 3, 0.5F, 2);
    state = bev::TrackManager::updateRecoveryState(state, rec_tracks, 3, 0.5F, 2);
    if (state.map_updates_paused) {
        std::cerr << "should resume after stable streak\n";
        return 1;
    }

    return 0;
}
