#include "tracking/keypoint_initializer.hpp"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    cv::Mat gray(120, 120, CV_8UC1, cv::Scalar(0));
    cv::rectangle(gray, cv::Point(20, 20), cv::Point(40, 40), cv::Scalar(255), cv::FILLED);
    cv::rectangle(gray, cv::Point(70, 70), cv::Point(90, 90), cv::Scalar(255), cv::FILLED);

    const auto points = bev::KeypointInitializer::initializeShiTomasi(gray, 20, 0.01, 5.0);
    if (points.empty()) {
        std::cerr << "expected non-empty Shi-Tomasi points\n";
        return 1;
    }

    const auto grid = bev::KeypointInitializer::initializeGrid(gray.size(), 3, 4, 10);
    if (grid.size() != 12U) {
        std::cerr << "grid point count mismatch\n";
        return 1;
    }

    return 0;
}