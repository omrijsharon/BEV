#include "web/dashboard_renderer.hpp"

#include <iostream>

#include <opencv2/core.hpp>

int main() {
    cv::Mat frame(120, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    const bev::DashboardStatus status{15, 42, 30, true};

    bev::DashboardRenderer::renderStatusOverlay(frame, status);

    const cv::Vec3b pixel = frame.at<cv::Vec3b>(10, 10);
    if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) {
        std::cerr << "overlay did not modify frame\n";
        return 1;
    }

    cv::Mat map_frame(160, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    const bev::MapDashboardStatus map_status{12.0, 0.8, 0.25, false};
    bev::DashboardRenderer::renderMapOverlay(map_frame, map_status);
    const cv::Vec3b map_pixel = map_frame.at<cv::Vec3b>(10, 10);
    if (map_pixel[0] == 0 && map_pixel[1] == 0 && map_pixel[2] == 0) {
        std::cerr << "map overlay did not modify frame\n";
        return 1;
    }

    return 0;
}
