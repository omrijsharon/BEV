#pragma once

#include <opencv2/core.hpp>

namespace bev {

struct TemplateMatchResult {
    cv::Point2f global_top_left{0.0F, 0.0F};
    float score{-1.0F};
    bool valid{false};
};

cv::Point2f localToGlobal(const cv::Rect& search_window, const cv::Point2f& local_match);
TemplateMatchResult matchTemplateInWindow(
    const cv::Mat& frame_gray,
    const cv::Mat& template_gray,
    const cv::Rect& search_window,
    int stride = 1);
TemplateMatchResult coarseFineMatch(
    const cv::Mat& frame_gray,
    const cv::Mat& template_gray,
    const cv::Rect& coarse_window,
    int fine_window_size,
    int coarse_stride = 2);

}  // namespace bev
