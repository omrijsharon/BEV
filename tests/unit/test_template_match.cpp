#include "tracking/template_tracker.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    cv::Mat frame(120, 160, CV_8UC1, cv::Scalar(10));
    cv::Rect target(70, 50, 16, 16);
    cv::Mat patch(16, 16, CV_8UC1, cv::Scalar(20));
    cv::line(patch, cv::Point(0, 0), cv::Point(15, 15), cv::Scalar(240), 1);
    cv::line(patch, cv::Point(0, 15), cv::Point(15, 0), cv::Scalar(180), 1);
    patch.copyTo(frame(target));

    cv::Mat templ = frame(target).clone();

    const cv::Rect coarse_window(40, 20, 80, 80);
    const auto coarse = bev::matchTemplateInWindow(frame, templ, coarse_window, 2);
    if (!coarse.valid) {
        std::cerr << "coarse match invalid\n";
        return 1;
    }

    const auto fine = bev::coarseFineMatch(frame, templ, coarse_window, 40, 2);
    if (!fine.valid) {
        std::cerr << "fine match invalid\n";
        return 1;
    }

    if (std::abs(fine.global_top_left.x - static_cast<float>(target.x)) > 1.0F ||
        std::abs(fine.global_top_left.y - static_cast<float>(target.y)) > 1.0F) {
        std::cerr << "fine match location mismatch\n";
        return 1;
    }

    return 0;
}
