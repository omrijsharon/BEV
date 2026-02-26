#include "map/minimap_builder.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace bev {

bool MinimapBuilder::initialize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return false;
    }

    canvas_ = cv::Mat::zeros(height, width, CV_8UC1);
    weight_ = cv::Mat::zeros(height, width, CV_32FC1);
    map_offset_transform_ = cv::Mat::eye(3, 3, CV_64F);
    return true;
}

std::vector<cv::Point2f> MinimapBuilder::frameCorners(const cv::Size& size) {
    return {
        {0.0F, 0.0F},
        {static_cast<float>(size.width - 1), 0.0F},
        {static_cast<float>(size.width - 1), static_cast<float>(size.height - 1)},
        {0.0F, static_cast<float>(size.height - 1)}
    };
}

void MinimapBuilder::ensureCanvasContains(const std::vector<cv::Point2f>& corners) {
    if (canvas_.empty() || corners.empty()) {
        return;
    }

    float min_x = corners[0].x;
    float min_y = corners[0].y;
    float max_x = corners[0].x;
    float max_y = corners[0].y;
    for (const auto& c : corners) {
        min_x = std::min(min_x, c.x);
        min_y = std::min(min_y, c.y);
        max_x = std::max(max_x, c.x);
        max_y = std::max(max_y, c.y);
    }

    const int add_left = std::max(0, static_cast<int>(std::ceil(-min_x)));
    const int add_top = std::max(0, static_cast<int>(std::ceil(-min_y)));
    const int add_right = std::max(0, static_cast<int>(std::ceil(max_x - (canvas_.cols - 1))));
    const int add_bottom = std::max(0, static_cast<int>(std::ceil(max_y - (canvas_.rows - 1))));

    if (add_left == 0 && add_top == 0 && add_right == 0 && add_bottom == 0) {
        return;
    }

    cv::Mat new_canvas;
    cv::copyMakeBorder(canvas_, new_canvas, add_top, add_bottom, add_left, add_right, cv::BORDER_CONSTANT, 0);
    canvas_ = new_canvas;

    cv::Mat new_weight;
    cv::copyMakeBorder(weight_, new_weight, add_top, add_bottom, add_left, add_right, cv::BORDER_CONSTANT, 0);
    weight_ = new_weight;

    cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
    T.at<double>(0, 2) = static_cast<double>(add_left);
    T.at<double>(1, 2) = static_cast<double>(add_top);
    map_offset_transform_ = T * map_offset_transform_;
}

bool MinimapBuilder::addFrame(const cv::Mat& bev_gray, const cv::Mat& H_map_from_frame) {
    if (canvas_.empty() || bev_gray.empty() || bev_gray.type() != CV_8UC1 || H_map_from_frame.empty()) {
        return false;
    }

    cv::Mat H_frame_to_canvas = map_offset_transform_ * H_map_from_frame;

    std::vector<cv::Point2f> projected_corners;
    cv::perspectiveTransform(frameCorners(bev_gray.size()), projected_corners, H_frame_to_canvas);
    ensureCanvasContains(projected_corners);

    H_frame_to_canvas = map_offset_transform_ * H_map_from_frame;

    cv::Mat warped_frame;
    cv::warpPerspective(bev_gray, warped_frame, H_frame_to_canvas, canvas_.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    cv::Mat mask;
    cv::threshold(warped_frame, mask, 0, 255, cv::THRESH_BINARY);

    for (int y = 0; y < canvas_.rows; ++y) {
        const auto* mask_row = mask.ptr<unsigned char>(y);
        const auto* src_row = warped_frame.ptr<unsigned char>(y);
        auto* dst_row = canvas_.ptr<unsigned char>(y);
        auto* w_row = weight_.ptr<float>(y);
        for (int x = 0; x < canvas_.cols; ++x) {
            if (mask_row[x] == 0U) {
                continue;
            }
            const float w_old = w_row[x];
            const float w_new = w_old + 1.0F;
            const float blended = (dst_row[x] * w_old + src_row[x]) / w_new;
            dst_row[x] = static_cast<unsigned char>(std::round(blended));
            w_row[x] = w_new;
        }
    }

    return true;
}

MinimapBuilder::Stats MinimapBuilder::stats() const {
    Stats out;
    if (canvas_.empty() || weight_.empty()) {
        return out;
    }

    int non_zero = 0;
    double weight_sum = 0.0;
    for (int y = 0; y < canvas_.rows; ++y) {
        const auto* c_row = canvas_.ptr<unsigned char>(y);
        const auto* w_row = weight_.ptr<float>(y);
        for (int x = 0; x < canvas_.cols; ++x) {
            if (c_row[x] == 0U) {
                continue;
            }
            ++non_zero;
            weight_sum += static_cast<double>(w_row[x]);
        }
    }

    const int total = canvas_.rows * canvas_.cols;
    if (total > 0) {
        out.coverage_ratio = static_cast<double>(non_zero) / static_cast<double>(total);
    }
    if (non_zero > 0) {
        out.mean_weight = weight_sum / static_cast<double>(non_zero);
    }
    return out;
}

}  // namespace bev
