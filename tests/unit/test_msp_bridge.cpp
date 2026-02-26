#include "msp/msp_bridge.hpp"

#include <cmath>
#include <iostream>

int main() {
    bev::MSPBridge bridge(16);

    bridge.pushAttitudeSample({100, 0.0, 0.0, 0.0});
    bridge.pushAttitudeSample({200, 1.0, 2.0, 3.0});

    const auto nearest = bridge.getNearestSample(170);
    if (!nearest.has_value() || nearest->timestamp_ns != 200) {
        std::cerr << "nearest sample selection failed\n";
        return 1;
    }

    const auto interp = bridge.getInterpolatedSample(150);
    if (!interp.has_value()) {
        std::cerr << "interpolated sample missing\n";
        return 1;
    }

    if (std::abs(interp->roll_rad - 0.5) > 1e-9) {
        std::cerr << "roll interpolation failed\n";
        return 1;
    }
    if (std::abs(interp->pitch_rad - 1.0) > 1e-9) {
        std::cerr << "pitch interpolation failed\n";
        return 1;
    }
    if (std::abs(interp->yaw_rad - 1.5) > 1e-9) {
        std::cerr << "yaw interpolation failed\n";
        return 1;
    }

    return 0;
}