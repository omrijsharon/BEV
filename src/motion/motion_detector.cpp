#include "motion/motion_detector.hpp"

#include <cmath>
#include <cstdint>
#include <opencv2/imgproc.hpp>

namespace bev {

void computeSignedDiff(const cv::Mat& prev_gray, const cv::Mat& curr_gray, cv::Mat& out_signed_diff_16s) {
    CV_Assert(!prev_gray.empty() && !curr_gray.empty());
    CV_Assert(prev_gray.type() == CV_8UC1 && curr_gray.type() == CV_8UC1);
    CV_Assert(prev_gray.size() == curr_gray.size());

    cv::Mat prev_16s;
    cv::Mat curr_16s;
    prev_gray.convertTo(prev_16s, CV_16S);
    curr_gray.convertTo(curr_16s, CV_16S);
    cv::subtract(curr_16s, prev_16s, out_signed_diff_16s, cv::noArray(), CV_16S);
}

cv::Mat computeSignedDiff(const cv::Mat& prev_gray, const cv::Mat& curr_gray) {
    cv::Mat diff;
    computeSignedDiff(prev_gray, curr_gray, diff);
    return diff;
}

void applyMotionDeadzoneInPlace(cv::Mat& signed_diff_16s, int threshold) {
    CV_Assert(signed_diff_16s.type() == CV_16S);
    for (int y = 0; y < signed_diff_16s.rows; ++y) {
        auto* row = signed_diff_16s.ptr<int16_t>(y);
        for (int x = 0; x < signed_diff_16s.cols; ++x) {
            if (std::abs(row[x]) <= threshold) {
                row[x] = 0;
            }
        }
    }
}

cv::Mat applyMotionDeadzone(const cv::Mat& signed_diff_16s, int threshold) {
    cv::Mat output = signed_diff_16s.clone();
    applyMotionDeadzoneInPlace(output, threshold);
    return output;
}

void renderMotionOverlay(const cv::Mat& base_bgr, const cv::Mat& signed_diff_16s, float alpha, cv::Mat& out) {
    CV_Assert(base_bgr.type() == CV_8UC3);
    CV_Assert(signed_diff_16s.type() == CV_16S);
    CV_Assert(base_bgr.rows == signed_diff_16s.rows && base_bgr.cols == signed_diff_16s.cols);

    if (out.empty() || out.size() != base_bgr.size() || out.type() != base_bgr.type()) {
        out.create(base_bgr.size(), base_bgr.type());
    }
    base_bgr.copyTo(out);

    const float clamped_alpha = std::max(0.0F, std::min(1.0F, alpha));
    for (int y = 0; y < out.rows; ++y) {
        const auto* diff_row = signed_diff_16s.ptr<int16_t>(y);
        auto* out_row = out.ptr<cv::Vec3b>(y);
        for (int x = 0; x < out.cols; ++x) {
            const int16_t d = diff_row[x];
            if (d == 0) {
                continue;
            }

            cv::Vec3f overlay(0.0F, 0.0F, 0.0F);
            const float intensity = std::min(1.0F, std::abs(static_cast<float>(d)) / 255.0F);
            if (d < 0) {
                overlay[0] = 255.0F * intensity;
            } else {
                overlay[2] = 255.0F * intensity;
            }
            cv::Vec3f base(out_row[x][0], out_row[x][1], out_row[x][2]);
            const cv::Vec3f blended = base * (1.0F - clamped_alpha) + overlay * clamped_alpha;
            out_row[x] = cv::Vec3b(
                static_cast<uint8_t>(std::round(blended[0])),
                static_cast<uint8_t>(std::round(blended[1])),
                static_cast<uint8_t>(std::round(blended[2])));
        }
    }
}

cv::Mat renderMotionOverlay(const cv::Mat& base_bgr, const cv::Mat& signed_diff_16s, float alpha) {
    cv::Mat out;
    renderMotionOverlay(base_bgr, signed_diff_16s, alpha, out);
    return out;
}

}  // namespace bev
