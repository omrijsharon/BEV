#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace bev {

struct AttitudeSample {
    int64_t timestamp_ns{0};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};
};

struct FramePacket {
    int64_t timestamp_ns{0};
    cv::Mat raw_bgr;
    cv::Mat undistorted_bgr;
    cv::Mat bev_gray;
};

struct TrackPoint {
    int id{-1};
    cv::Point2f pos{0.0F, 0.0F};
    cv::Point2f vel_px_s{0.0F, 0.0F};
    float confidence{0.0F};
    bool valid{false};
};

struct RegistrationResult {
    cv::Mat H_imu_bev;
    cv::Mat H_refine;
    cv::Mat H_final;
    int inliers{0};
    float inlier_ratio{0.0F};
    float reproj_rmse{0.0F};
};

}  // namespace bev