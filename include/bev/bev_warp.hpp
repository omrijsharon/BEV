#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "core/types.hpp"

namespace bev {

class BEVWarp {
public:
    BEVWarp() = default;
    explicit BEVWarp(const cv::Mat& camera_matrix);

    bool setCameraMatrix(const cv::Mat& camera_matrix);
    void setReferenceYaw(double yaw_rad);

    cv::Mat computeHomography(const AttitudeSample& sample) const;
    cv::Mat computeHomographyFromRotation(const cv::Matx33d& R_cam_world) const;
    bool warpToBEV(const cv::Mat& src, cv::Mat& dst, const AttitudeSample& sample, int interpolation = cv::INTER_LINEAR) const;
    bool warpToBEV(const cv::Mat& src, cv::Mat& dst, const cv::Matx33d& R_cam_world, int interpolation = cv::INTER_LINEAR) const;

private:
    cv::Mat K_;
    cv::Mat K_inv_;
    double ref_yaw_rad_{0.0};
};

}  // namespace bev
