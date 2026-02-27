#include "ipc/spsc_ring.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
    using Clock = std::chrono::steady_clock;
    constexpr int kIters = 200000;

    bev::ipc::SpscRing<uint64_t> ring(1024);
    if (!ring.valid()) {
        std::cerr << "ring invalid\n";
        return 1;
    }

    std::vector<int64_t> lat_ns;
    lat_ns.reserve(kIters);

    for (int i = 0; i < kIters; ++i) {
        const auto t0 = Clock::now();
        const uint64_t stamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
        (void)ring.pushDropOldest(stamp);

        uint64_t out = 0;
        while (!ring.pop(out)) {
        }
        const auto t1 = Clock::now();
        const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1.time_since_epoch()).count();
        lat_ns.push_back(now_ns - static_cast<int64_t>(out));
    }

    std::sort(lat_ns.begin(), lat_ns.end());
    auto pct = [&](double p) -> int64_t {
        const std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(lat_ns.size() - 1));
        return lat_ns[idx];
    };

    std::cout << "bench_ipc_iterations " << kIters << "\n";
    std::cout << "latency_ns_p50 " << pct(0.50) << "\n";
    std::cout << "latency_ns_p95 " << pct(0.95) << "\n";
    std::cout << "latency_ns_p99 " << pct(0.99) << "\n";
    std::cout << "drops " << ring.dropCount() << "\n";
    return 0;
}
