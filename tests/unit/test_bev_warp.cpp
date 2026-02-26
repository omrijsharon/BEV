#include "bev/bev_warp.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

int main() {
    const cv::Mat K = (cv::Mat_<double>(3, 3) <<
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0);

    bev::BEVWarp warp(K);
    const bev::AttitudeSample sample{0, 0.0, 0.0, 0.0};
    const cv::Mat H = warp.computeHomography(sample);

    if (H.rows != 3 || H.cols != 3) {
        std::cerr << "homography shape invalid\n";
        return 1;
    }

    const double h00 = H.at<double>(0, 0);
    const double h11 = H.at<double>(1, 1);
    const double h22 = H.at<double>(2, 2);
    if (std::abs(h00 - 1.0) > 1e-9 || std::abs(h11 - 1.0) > 1e-9 || std::abs(h22 - 1.0) > 1e-9) {
        std::cerr << "identity homography expectation failed\n";
        return 1;
    }

    return 0;
}