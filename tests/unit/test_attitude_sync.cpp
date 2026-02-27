#include "msp/attitude_sync.hpp"

#include <iostream>

int main() {
    bev::AttitudeSyncBuffer buf(16);
    bev::SyncPolicy policy{};
    policy.max_attitude_age_ns = 10000000;
    policy.max_interp_gap_ns = 10000000;
    policy.startup_min_buffer = 2;

    auto a = bev::ipc::makeAttitudeDesc();
    a.timestamp_ns = 1000;
    a.roll_rad = 0.0F;
    a.pitch_rad = 0.0F;
    a.yaw_rad = 0.0F;
    auto b = bev::ipc::makeAttitudeDesc();
    b.timestamp_ns = 2000;
    b.roll_rad = 1.0F;
    b.pitch_rad = 1.0F;
    b.yaw_rad = 1.0F;
    buf.push(a);
    buf.push(b);

    const auto mid = buf.resolve(1500, policy);
    if (!mid.ok || !mid.interpolated) {
        std::cerr << "expected interpolated resolve\n";
        return 1;
    }
    if (mid.attitude.roll_rad < 0.49F || mid.attitude.roll_rad > 0.51F) {
        std::cerr << "interpolation value mismatch\n";
        return 1;
    }

    const auto near = buf.resolve(2050, policy);
    if (!near.ok) {
        std::cerr << "nearest resolve expected ok\n";
        return 1;
    }

    policy.max_attitude_age_ns = 10;
    const auto stale = buf.resolve(100000, policy);
    if (stale.ok) {
        std::cerr << "stale resolve should fail\n";
        return 1;
    }

    return 0;
}
