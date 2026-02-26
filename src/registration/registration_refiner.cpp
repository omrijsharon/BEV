#include "registration/registration_refiner.hpp"

#include <cmath>

#include <opencv2/calib3d.hpp>

namespace bev {

RegistrationRefiner::RegistrationRefiner(RefinerConfig config)
    : config_(config) {}

bool RegistrationRefiner::estimateHomography(
    const std::vector<cv::Point2f>& src_points,
    const std::vector<cv::Point2f>& dst_points,
    RegistrationResult& out_result) const {
    out_result = RegistrationResult{};
    if (src_points.size() != dst_points.size() || src_points.size() < 4) {
        return false;
    }

    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(
        src_points,
        dst_points,
        cv::RANSAC,
        config_.ransac_reproj_threshold_px,
        inlier_mask,
        config_.ransac_max_iters,
        config_.ransac_confidence);

    if (H.empty() || inlier_mask.empty()) {
        return false;
    }

    int inlier_count = 0;
    const int mask_count = static_cast<int>(inlier_mask.total());
    for (int i = 0; i < mask_count; ++i) {
        if (inlier_mask.at<unsigned char>(i) != 0U) {
            ++inlier_count;
        }
    }

    const float inlier_ratio = static_cast<float>(inlier_count) / static_cast<float>(src_points.size());
    if (inlier_count < config_.min_inliers || inlier_ratio < config_.min_inlier_ratio) {
        return false;
    }

    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(src_points, projected, H);

    double sq_error_sum = 0.0;
    int error_count = 0;
    for (int i = 0; i < static_cast<int>(src_points.size()); ++i) {
        if (inlier_mask.at<unsigned char>(i) == 0U) {
            continue;
        }
        const cv::Point2f diff = projected[i] - dst_points[i];
        sq_error_sum += static_cast<double>(diff.x * diff.x + diff.y * diff.y);
        ++error_count;
    }
    const float rmse = (error_count > 0) ? static_cast<float>(std::sqrt(sq_error_sum / static_cast<double>(error_count))) : 0.0F;

    out_result.H_refine = H;
    out_result.H_final = H;
    out_result.inliers = inlier_count;
    out_result.inlier_ratio = inlier_ratio;
    out_result.reproj_rmse = rmse;
    return true;
}

}  // namespace bev
