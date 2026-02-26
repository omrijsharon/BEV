#include "bev/bev_warp.hpp"

#include <opencv2/imgproc.hpp>

#include "core/math_utils.hpp"

namespace bev {

BEVWarp::BEVWarp(const cv::Mat& camera_matrix) {
    setCameraMatrix(camera_matrix);
}

bool BEVWarp::setCameraMatrix(const cv::Mat& camera_matrix) {
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        return false;
    }
    camera_matrix.convertTo(K_, CV_64F);
    K_inv_ = K_.inv();
    return true;
}

void BEVWarp::setReferenceYaw(double yaw_rad) {
    ref_yaw_rad_ = yaw_rad;
}

cv::Mat BEVWarp::computeHomography(const AttitudeSample& sample) const {
    const double relative_yaw = sample.yaw_rad - ref_yaw_rad_;
    const cv::Matx33d R_cam_world = eulerToRotationMatrix(sample.roll_rad, sample.pitch_rad, relative_yaw);
    return computeHomographyFromRotation(R_cam_world);
}

cv::Mat BEVWarp::computeHomographyFromRotation(const cv::Matx33d& R_cam_world) const {
    if (K_.empty() || K_inv_.empty()) {
        return cv::Mat::eye(3, 3, CV_64F);
    }

    const cv::Mat R_inv = cv::Mat(R_cam_world).t();
    return K_ * R_inv * K_inv_;
}

bool BEVWarp::warpToBEV(const cv::Mat& src, cv::Mat& dst, const AttitudeSample& sample, int interpolation) const {
    if (src.empty()) {
        return false;
    }

    const cv::Mat H = computeHomography(sample);
    cv::warpPerspective(src, dst, H, src.size(), interpolation, cv::BORDER_CONSTANT);
    return !dst.empty();
}

bool BEVWarp::warpToBEV(const cv::Mat& src, cv::Mat& dst, const cv::Matx33d& R_cam_world, int interpolation) const {
    if (src.empty()) {
        return false;
    }

    const cv::Mat H = computeHomographyFromRotation(R_cam_world);
    cv::warpPerspective(src, dst, H, src.size(), interpolation, cv::BORDER_CONSTANT);
    return !dst.empty();
}

}  // namespace bev
