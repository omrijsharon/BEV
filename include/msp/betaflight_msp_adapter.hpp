#pragma once

#include <cstdint>

#include "core/types.hpp"

#ifdef BEV_ENABLE_ARDUINO_MSP
#include "msp/BetaflightMSP.h"
#endif

namespace bev {

class BetaflightMSPAdapter {
public:
    BetaflightMSPAdapter() = default;

#ifdef BEV_ENABLE_ARDUINO_MSP
    bool initialize(Stream& serial_port);
#else
    bool initialize();
#endif

    bool pollAttitudeSample(int64_t timestamp_ns, AttitudeSample& out_sample);
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_{false};

#ifdef BEV_ENABLE_ARDUINO_MSP
    BetaflightMSP msp_;
#endif
};

}  // namespace bev
