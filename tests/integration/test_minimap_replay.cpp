#include "map/minimap_builder.hpp"

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Mat makePatch(int w, int h) {
    cv::Mat img(h, w, CV_8UC1, cv::Scalar(30));
    cv::RNG rng(777);
    for (int i = 0; i < 50; ++i) {
        cv::Point c(rng.uniform(0, w), rng.uniform(0, h));
        cv::circle(img, c, rng.uniform(2, 9), cv::Scalar(rng.uniform(90, 230)), 1, cv::LINE_AA);
    }
    return img;
}

cv::Mat HTranslate(double tx, double ty) {
    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    H.at<double>(0, 2) = tx;
    H.at<double>(1, 2) = ty;
    return H;
}

} // namespace

int main() {
    bev::MinimapBuilder map;
    if (!map.initialize(160, 120)) {
        return 1;
    }

    const cv::Mat patch = makePatch(80, 60);

    for (int i = 0; i < 12; ++i) {
        const double tx = -30.0 + i * 8.0;
        const double ty = -20.0 + i * 4.0;
        if (!map.addFrame(patch, HTranslate(tx, ty))) {
            std::cerr << "failed to add frame at step " << i << "\n";
            return 1;
        }
    }

    const auto s = map.stats();
    if (s.coverage_ratio < 0.05) {
        std::cerr << "coverage ratio too low: " << s.coverage_ratio << "\n";
        return 1;
    }
    if (s.mean_weight < 1.0) {
        std::cerr << "mean weight too low: " << s.mean_weight << "\n";
        return 1;
    }

    return 0;
}