#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace bev {

class MinimapBuilder {
public:
    struct Stats {
        double coverage_ratio{0.0};   // non-zero pixels / total pixels
        double mean_weight{0.0};      // mean blend weight over non-zero map pixels
    };

    MinimapBuilder() = default;

    bool initialize(int width, int height);
    bool addFrame(const cv::Mat& bev_gray, const cv::Mat& H_map_from_frame);
    Stats stats() const;

    const cv::Mat& canvas() const { return canvas_; }
    const cv::Mat& mapOffsetTransform() const { return map_offset_transform_; }

private:
    void ensureCanvasContains(const std::vector<cv::Point2f>& corners);
    static std::vector<cv::Point2f> frameCorners(const cv::Size& size);

    cv::Mat canvas_;               // CV_8UC1
    cv::Mat weight_;               // CV_32FC1
    cv::Mat map_offset_transform_; // CV_64F, 3x3
};

}  // namespace bev
