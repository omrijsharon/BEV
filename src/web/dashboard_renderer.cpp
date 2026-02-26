#include "web/dashboard_renderer.hpp"

#include <opencv2/imgproc.hpp>

namespace bev {

void DashboardRenderer::renderStatusOverlay(cv::Mat& bgr_frame, const DashboardStatus& status) {
    if (bgr_frame.empty()) {
        return;
    }

    const cv::Scalar bg(20, 20, 20);
    const cv::Scalar fg(220, 220, 220);
    const cv::Scalar ok(60, 200, 60);
    const cv::Scalar bad(40, 40, 220);

    cv::rectangle(bgr_frame, cv::Rect(8, 8, 260, 90), bg, cv::FILLED);
    cv::putText(bgr_frame, "FPS: " + std::to_string(status.fps), cv::Point(16, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(bgr_frame, "Tracks: " + std::to_string(status.valid_tracks), cv::Point(16, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(bgr_frame, "Inliers: " + std::to_string(status.inliers), cv::Point(16, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(
        bgr_frame,
        std::string("MSP: ") + (status.msp_fresh ? "fresh" : "stale"),
        cv::Point(16, 90),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        status.msp_fresh ? ok : bad,
        1,
        cv::LINE_AA);
}

void DashboardRenderer::renderMapOverlay(cv::Mat& bgr_frame, const MapDashboardStatus& status) {
    if (bgr_frame.empty()) {
        return;
    }

    const cv::Scalar bg(20, 20, 20);
    const cv::Scalar fg(220, 220, 220);
    const cv::Scalar warn(30, 180, 240);
    const cv::Scalar ok(60, 200, 60);

    cv::rectangle(bgr_frame, cv::Rect(8, 8, 320, 110), bg, cv::FILLED);
    cv::putText(bgr_frame, "Map Drift(px): " + std::to_string(static_cast<int>(status.drift_px)),
                cv::Point(16, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(bgr_frame, "Confidence: " + std::to_string(static_cast<int>(status.confidence * 100.0)) + "%",
                cv::Point(16, 52), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(bgr_frame, "Coverage: " + std::to_string(static_cast<int>(status.coverage_ratio * 100.0)) + "%",
                cv::Point(16, 74), cv::FONT_HERSHEY_SIMPLEX, 0.5, fg, 1, cv::LINE_AA);
    cv::putText(bgr_frame,
                std::string("Map Updates: ") + (status.map_paused ? "PAUSED" : "RUNNING"),
                cv::Point(16, 96), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                status.map_paused ? warn : ok, 1, cv::LINE_AA);
}

}  // namespace bev
