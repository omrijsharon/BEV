#include "core/math_utils.hpp"

#include <cmath>
#include <iostream>

int main() {
    const double pi = 3.14159265358979323846;
    const double rad180 = bev::degreesToRadians(180.0);
    if (std::abs(rad180 - pi) > 1e-9) {
        std::cerr << "degreesToRadians failed\n";
        return 1;
    }

    const double rad10 = bev::deciDegreesToRadians(10.0);
    const double expected10 = bev::degreesToRadians(1.0);
    if (std::abs(rad10 - expected10) > 1e-9) {
        std::cerr << "deciDegreesToRadians failed\n";
        return 1;
    }

    const auto R = bev::eulerToRotationMatrix(0.0, 0.0, 0.0);
    if (std::abs(R(0, 0) - 1.0) > 1e-9 || std::abs(R(1, 1) - 1.0) > 1e-9 || std::abs(R(2, 2) - 1.0) > 1e-9) {
        std::cerr << "eulerToRotationMatrix identity failed\n";
        return 1;
    }

    const auto Rcf = bev::bodyToOpenCVCameraForwardMatrix();
    const auto Rc0 = bev::composeCameraWorldRotation(0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0);
    if (std::abs(Rc0(0, 0) - Rcf(0, 0)) > 1e-9 || std::abs(Rc0(2, 0) - Rcf(2, 0)) > 1e-9) {
        std::cerr << "composeCameraWorldRotation base mapping failed\n";
        return 1;
    }

    const auto Rneg = bev::composeCameraWorldRotation(0.0, 0.0, bev::degreesToRadians(20.0), true, 0.0, 0.0, 0.0);
    const auto Rpos = bev::composeCameraWorldRotation(0.0, 0.0, bev::degreesToRadians(20.0), false, 0.0, 0.0, 0.0);
    if (std::abs(Rneg(0, 0) - Rpos(0, 0)) < 1e-6 && std::abs(Rneg(0, 1) - Rpos(0, 1)) < 1e-6) {
        std::cerr << "negate_yaw did not change composed rotation\n";
        return 1;
    }

    return 0;
}
