#include "motion/motion_detector.hpp"

#include <iostream>
#include <opencv2/core.hpp>

int main() {
    cv::Mat diff = (cv::Mat_<int16_t>(1, 5) << -40, -20, 0, 20, 40);
    const cv::Mat out = bev::applyMotionDeadzone(diff, 30);

    if (out.at<int16_t>(0, 0) != -40) return 1;
    if (out.at<int16_t>(0, 1) != 0) return 1;
    if (out.at<int16_t>(0, 2) != 0) return 1;
    if (out.at<int16_t>(0, 3) != 0) return 1;
    if (out.at<int16_t>(0, 4) != 40) return 1;
    cv::Mat inplace = diff.clone();
    bev::applyMotionDeadzoneInPlace(inplace, 30);
    if (inplace.at<int16_t>(0, 1) != 0) return 1;

    cv::Mat prev = (cv::Mat_<uint8_t>(1, 3) << 100, 100, 100);
    cv::Mat curr = (cv::Mat_<uint8_t>(1, 3) << 80, 100, 140);
    cv::Mat signed_diff = bev::computeSignedDiff(prev, curr);
    if (signed_diff.at<int16_t>(0, 0) != -20) return 1;
    if (signed_diff.at<int16_t>(0, 1) != 0) return 1;
    if (signed_diff.at<int16_t>(0, 2) != 40) return 1;

    cv::Mat base(1, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat overlay;
    bev::renderMotionOverlay(base, signed_diff, 1.0F, overlay);
    const cv::Vec3b neg_pixel = overlay.at<cv::Vec3b>(0, 0);
    const cv::Vec3b pos_pixel = overlay.at<cv::Vec3b>(0, 2);
    if (neg_pixel[0] == 0) return 1;  // blue on negative
    if (pos_pixel[2] == 0) return 1;  // red on positive

    return 0;
}
