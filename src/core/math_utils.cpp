#include "core/math_utils.hpp"

#include <cmath>

namespace bev {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double degreesToRadians(double degrees) {
    return degrees * (kPi / 180.0);
}

double deciDegreesToRadians(double deci_degrees) {
    return (deci_degrees * 0.1) * (kPi / 180.0);
}

cv::Matx33d eulerToRotationMatrix(double roll_rad, double pitch_rad, double yaw_rad) {
    const double cr = std::cos(roll_rad);
    const double sr = std::sin(roll_rad);
    const double cp = std::cos(pitch_rad);
    const double sp = std::sin(pitch_rad);
    const double cy = std::cos(yaw_rad);
    const double sy = std::sin(yaw_rad);

    // ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return cv::Matx33d(
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr);
}

cv::Matx33d matMul(const cv::Matx33d& a, const cv::Matx33d& b) {
    return a * b;
}

cv::Matx33d bodyToOpenCVCameraForwardMatrix() {
    // Body frame: x=forward, y=right, z=up
    // OpenCV camera frame: x=right, y=down, z=forward
    // This matrix maps body vectors into a front-facing OpenCV camera frame.
    return cv::Matx33d(
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0,
        1.0, 0.0, 0.0);
}

cv::Matx33d cameraMountRotationBodyFrame(double roll_deg, double pitch_deg, double yaw_deg) {
    return eulerToRotationMatrix(
        degreesToRadians(roll_deg),
        degreesToRadians(pitch_deg),
        degreesToRadians(yaw_deg));
}

cv::Matx33d composeCameraWorldRotation(
    double body_roll_rad,
    double body_pitch_rad,
    double body_yaw_rad,
    bool negate_body_yaw,
    double mount_roll_deg,
    double mount_pitch_deg,
    double mount_yaw_deg) {
    const double yaw = negate_body_yaw ? -body_yaw_rad : body_yaw_rad;
    const cv::Matx33d R_body_world = eulerToRotationMatrix(body_roll_rad, body_pitch_rad, yaw);
    const cv::Matx33d R_mount_body = cameraMountRotationBodyFrame(mount_roll_deg, mount_pitch_deg, mount_yaw_deg);
    const cv::Matx33d R_cam_front_body = bodyToOpenCVCameraForwardMatrix();

    // Camera orientation in world = camera-from-front * mount-from-body * body-from-world
    return matMul(matMul(R_cam_front_body, R_mount_body), R_body_world);
}

}  // namespace bev
