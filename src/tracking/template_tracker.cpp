#include "tracking/template_tracker.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace bev {

cv::Point2f localToGlobal(const cv::Rect& search_window, const cv::Point2f& local_match) {
    return cv::Point2f(search_window.x + local_match.x, search_window.y + local_match.y);
}

namespace {

cv::Rect clampRectToFrame(const cv::Rect& r, const cv::Size& frame_size) {
    const int x0 = std::max(0, r.x);
    const int y0 = std::max(0, r.y);
    const int x1 = std::min(frame_size.width, r.x + r.width);
    const int y1 = std::min(frame_size.height, r.y + r.height);
    return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0));
}

}  // namespace

TemplateMatchResult matchTemplateInWindow(
    const cv::Mat& frame_gray,
    const cv::Mat& template_gray,
    const cv::Rect& search_window,
    int stride) {
    TemplateMatchResult out;

    if (frame_gray.empty() || template_gray.empty() || frame_gray.type() != CV_8UC1 || template_gray.type() != CV_8UC1) {
        return out;
    }
    if (stride < 1) {
        stride = 1;
    }

    cv::Rect window = clampRectToFrame(search_window, frame_gray.size());
    if (window.width < template_gray.cols || window.height < template_gray.rows) {
        return out;
    }

    const cv::Mat search_roi = frame_gray(window);
    cv::Mat search_for_match = search_roi;
    cv::Mat template_for_match = template_gray;

    if (stride > 1) {
        const cv::Size search_small(
            std::max(1, search_roi.cols / stride),
            std::max(1, search_roi.rows / stride));
        const cv::Size templ_small(
            std::max(1, template_gray.cols / stride),
            std::max(1, template_gray.rows / stride));

        cv::resize(search_roi, search_for_match, search_small, 0, 0, cv::INTER_AREA);
        cv::resize(template_gray, template_for_match, templ_small, 0, 0, cv::INTER_AREA);
    }

    if (search_for_match.cols < template_for_match.cols || search_for_match.rows < template_for_match.rows) {
        return out;
    }

    cv::Mat result;
    cv::matchTemplate(search_for_match, template_for_match, result, cv::TM_CCOEFF_NORMED);

    double max_val = 0.0;
    cv::Point max_loc;
    cv::minMaxLoc(result, nullptr, &max_val, nullptr, &max_loc);

    out.global_top_left = cv::Point2f(
        static_cast<float>(window.x + max_loc.x * stride),
        static_cast<float>(window.y + max_loc.y * stride));
    out.score = static_cast<float>(max_val);
    out.valid = true;
    return out;
}

TemplateMatchResult coarseFineMatch(
    const cv::Mat& frame_gray,
    const cv::Mat& template_gray,
    const cv::Rect& coarse_window,
    int fine_window_size,
    int coarse_stride) {
    TemplateMatchResult coarse = matchTemplateInWindow(frame_gray, template_gray, coarse_window, coarse_stride);
    if (!coarse.valid) {
        return coarse;
    }

    const int fine_half = std::max(1, fine_window_size / 2);
    cv::Rect fine_window(
        static_cast<int>(coarse.global_top_left.x) - fine_half,
        static_cast<int>(coarse.global_top_left.y) - fine_half,
        std::max(template_gray.cols + 2, fine_window_size),
        std::max(template_gray.rows + 2, fine_window_size));

    return matchTemplateInWindow(frame_gray, template_gray, fine_window, 1);
}

}  // namespace bev
