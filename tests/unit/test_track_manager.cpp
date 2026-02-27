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

    std::vector<cv::Point2f> prev_pts{
        {10.0F, 0.0F}, {0.0F, 10.0F}, {-10.0F, 0.0F}, {0.0F, -10.0F}};
    std::vector<cv::Point2f> curr_pts;
    std::vector<float> w{1.0F, 1.0F, 1.0F, 1.0F};
    curr_pts.reserve(prev_pts.size());
    const float dt = 0.1F;
    const float omega_gt = 0.6F; // rad/s
    const float theta = omega_gt * dt;
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    const cv::Point2f t(3.0F, -2.0F); // translation should not affect omega estimate
    for (const auto& p : prev_pts) {
        curr_pts.emplace_back(c * p.x - s * p.y + t.x, s * p.x + c * p.y + t.y);
    }
    const float omega_est = bev::TrackManager::estimateOmegaRadPerSec(prev_pts, curr_pts, dt, &w);
    if (std::abs(omega_est - omega_gt) > 0.03F) {
        std::cerr << "omega estimate mismatch est=" << omega_est << " gt=" << omega_gt << "\n";
        return 1;
    }

    return 0;
}
