#include "msp/betaflight_msp_adapter.hpp"

#include "core/math_utils.hpp"

namespace bev {

#ifdef BEV_ENABLE_ARDUINO_MSP
bool BetaflightMSPAdapter::initialize(Stream& serial_port) {
    msp_.begin(serial_port);
    initialized_ = true;
    return true;
}
#else
bool BetaflightMSPAdapter::initialize() {
    initialized_ = false;
    return false;
}
#endif

bool BetaflightMSPAdapter::pollAttitudeSample(int64_t timestamp_ns, AttitudeSample& out_sample) {
#ifdef BEV_ENABLE_ARDUINO_MSP
    if (!initialized_) {
        return false;
    }

    msp_attitude_t attitude{};
    if (!msp_.getAttitude(attitude)) {
        return false;
    }

    out_sample.timestamp_ns = timestamp_ns;
    out_sample.roll_rad = deciDegreesToRadians(static_cast<double>(attitude.roll));
    out_sample.pitch_rad = deciDegreesToRadians(static_cast<double>(attitude.pitch));
    out_sample.yaw_rad = degreesToRadians(static_cast<double>(attitude.yaw));
    return true;
#else
    (void)timestamp_ns;
    (void)out_sample;
    return false;
#endif
}

}  // namespace bev
