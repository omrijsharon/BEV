#pragma once

#include <opencv2/core.hpp>

namespace bev {

void computeSignedDiff(const cv::Mat& prev_gray, const cv::Mat& curr_gray, cv::Mat& out_signed_diff_16s);
cv::Mat computeSignedDiff(const cv::Mat& prev_gray, const cv::Mat& curr_gray);
void applyMotionDeadzoneInPlace(cv::Mat& signed_diff_16s, int threshold);
cv::Mat applyMotionDeadzone(const cv::Mat& signed_diff_16s, int threshold);
void renderMotionOverlay(const cv::Mat& base_bgr, const cv::Mat& signed_diff_16s, float alpha, cv::Mat& out_bgr);
cv::Mat renderMotionOverlay(const cv::Mat& base_bgr, const cv::Mat& signed_diff_16s, float alpha);

}  // namespace bev
