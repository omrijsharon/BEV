#pragma once

#include <opencv2/core.hpp>

namespace bev {

struct DashboardStatus {
    int fps{0};
    int valid_tracks{0};
    int inliers{0};
    bool msp_fresh{false};
};

struct MapDashboardStatus {
    double drift_px{0.0};
    double confidence{0.0};     // 0..1
    double coverage_ratio{0.0}; // 0..1
    bool map_paused{false};
};

class DashboardRenderer {
public:
    static void renderStatusOverlay(cv::Mat& bgr_frame, const DashboardStatus& status);
    static void renderMapOverlay(cv::Mat& bgr_frame, const MapDashboardStatus& status);
};

}  // namespace bev
