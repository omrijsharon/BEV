#include "map/minimap_builder.hpp"

#include <iostream>

#include <opencv2/core.hpp>

int main() {
    bev::MinimapBuilder builder;
    if (!builder.initialize(80, 80)) {
        std::cerr << "failed to initialize minimap\n";
        return 1;
    }

    cv::Mat frame(20, 20, CV_8UC1, cv::Scalar(120));

    cv::Mat H1 = cv::Mat::eye(3, 3, CV_64F);
    H1.at<double>(0, 2) = 10.0;
    H1.at<double>(1, 2) = 10.0;
    if (!builder.addFrame(frame, H1)) {
        std::cerr << "first frame insertion failed\n";
        return 1;
    }

    cv::Mat H2 = cv::Mat::eye(3, 3, CV_64F);
    H2.at<double>(0, 2) = -30.0;
    H2.at<double>(1, 2) = -20.0;
    if (!builder.addFrame(frame, H2)) {
        std::cerr << "second frame insertion failed\n";
        return 1;
    }

    if (builder.canvas().empty()) {
        std::cerr << "canvas should not be empty\n";
        return 1;
    }
    if (builder.canvas().cols < 80 || builder.canvas().rows < 80) {
        std::cerr << "canvas should expand or keep original size\n";
        return 1;
    }

    const auto stats = builder.stats();
    if (stats.coverage_ratio <= 0.0) {
        std::cerr << "coverage ratio should be positive\n";
        return 1;
    }
    if (stats.mean_weight <= 0.0) {
        std::cerr << "mean weight should be positive\n";
        return 1;
    }

    return 0;
}
