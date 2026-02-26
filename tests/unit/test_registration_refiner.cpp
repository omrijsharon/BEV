#include "registration/registration_refiner.hpp"

#include <iostream>
#include <vector>

int main() {
    std::vector<cv::Point2f> src{
        {0.0F, 0.0F}, {100.0F, 0.0F}, {0.0F, 100.0F}, {100.0F, 100.0F},
        {50.0F, 20.0F}, {20.0F, 60.0F}, {80.0F, 40.0F}, {40.0F, 80.0F}
    };

    std::vector<cv::Point2f> dst;
    dst.reserve(src.size());
    for (const auto& p : src) {
        dst.emplace_back(p.x + 12.0F, p.y - 7.0F);
    }

    bev::RegistrationRefiner refiner;
    bev::RegistrationResult result;
    if (!refiner.estimateHomography(src, dst, result)) {
        std::cerr << "homography estimation should succeed\n";
        return 1;
    }

    if (result.inliers < 8) {
        std::cerr << "expected all inliers for synthetic translation\n";
        return 1;
    }

    return 0;
}