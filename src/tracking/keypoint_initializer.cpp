#include "tracking/keypoint_initializer.hpp"

#include <opencv2/imgproc.hpp>

namespace bev {

std::vector<cv::Point2f> KeypointInitializer::initializeShiTomasi(
    const cv::Mat& gray,
    int max_points,
    double quality_level,
    double min_distance) {
    std::vector<cv::Point2f> points;
    if (gray.empty() || gray.type() != CV_8UC1 || max_points <= 0) {
        return points;
    }

    cv::goodFeaturesToTrack(gray, points, max_points, quality_level, min_distance);
    return points;
}

std::vector<cv::Point2f> KeypointInitializer::initializeGrid(
    const cv::Size& frame_size,
    int rows,
    int cols,
    int margin_px) {
    std::vector<cv::Point2f> points;
    if (frame_size.width <= 0 || frame_size.height <= 0 || rows <= 0 || cols <= 0) {
        return points;
    }

    const int usable_w = std::max(1, frame_size.width - 2 * margin_px);
    const int usable_h = std::max(1, frame_size.height - 2 * margin_px);

    for (int r = 0; r < rows; ++r) {
        const float y = static_cast<float>(margin_px) + (r + 0.5F) * (static_cast<float>(usable_h) / static_cast<float>(rows));
        for (int c = 0; c < cols; ++c) {
            const float x = static_cast<float>(margin_px) + (c + 0.5F) * (static_cast<float>(usable_w) / static_cast<float>(cols));
            points.emplace_back(x, y);
        }
    }

    return points;
}

}  // namespace bev
