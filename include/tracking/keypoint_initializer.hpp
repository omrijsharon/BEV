#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace bev {

class KeypointInitializer {
public:
    static std::vector<cv::Point2f> initializeShiTomasi(
        const cv::Mat& gray,
        int max_points,
        double quality_level = 0.01,
        double min_distance = 8.0);

    static std::vector<cv::Point2f> initializeGrid(
        const cv::Size& frame_size,
        int rows,
        int cols,
        int margin_px = 20);
};

}  // namespace bev
