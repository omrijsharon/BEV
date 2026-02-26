#pragma once

#include <opencv2/core.hpp>

namespace bev {

double degreesToRadians(double degrees);
double deciDegreesToRadians(double deci_degrees);
cv::Matx33d eulerToRotationMatrix(double roll_rad, double pitch_rad, double yaw_rad);
cv::Matx33d matMul(const cv::Matx33d& a, const cv::Matx33d& b);
cv::Matx33d bodyToOpenCVCameraForwardMatrix();
cv::Matx33d cameraMountRotationBodyFrame(double roll_deg, double pitch_deg, double yaw_deg);
cv::Matx33d composeCameraWorldRotation(
    double body_roll_rad,
    double body_pitch_rad,
    double body_yaw_rad,
    bool negate_body_yaw,
    double mount_roll_deg,
    double mount_pitch_deg,
    double mount_yaw_deg);

}  // namespace bev
