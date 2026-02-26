#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/types.hpp"

namespace bev {

struct RefinerConfig {
    double ransac_reproj_threshold_px{3.0};
    double ransac_confidence{0.995};
    int ransac_max_iters{2000};
    int min_inliers{8};
    double min_inlier_ratio{0.4};
};

class RegistrationRefiner {
public:
    explicit RegistrationRefiner(RefinerConfig config = {});

    bool estimateHomography(
        const std::vector<cv::Point2f>& src_points,
        const std::vector<cv::Point2f>& dst_points,
        RegistrationResult& out_result) const;

private:
    RefinerConfig config_;
};

}  // namespace bev
