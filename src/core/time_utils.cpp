#include "core/time_utils.hpp"

#include <chrono>

namespace bev {

int64_t nowSteadyNs() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

}  // namespace bev