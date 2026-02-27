#include "ipc/spsc_ring.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

namespace {

struct Token {
    uint64_t seq{0};
    int64_t t0{0};
    int64_t t1{0};
    int64_t t2{0};
    int64_t t3{0};
};

inline int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline void syntheticWork(int n) {
    volatile int acc = 0;
    for (int i = 0; i < n; ++i) {
        acc += (i * 3) ^ (i >> 1);
    }
    (void)acc;
}

int64_t pct(std::vector<int64_t>& v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1));
    return v[idx];
}

}  // namespace

int main() {
    constexpr int kCount = 50000;
    bev::ipc::SpscRing<Token> q1(1024);
    bev::ipc::SpscRing<Token> q2(1024);
    bev::ipc::SpscRing<Token> q3(1024);
    if (!q1.valid() || !q2.valid() || !q3.valid()) {
        std::cerr << "ring init failed\n";
        return 1;
    }

    std::atomic<bool> prod_done{false};
    std::atomic<bool> s1_done{false};
    std::atomic<bool> s2_done{false};

    std::vector<int64_t> l1;
    std::vector<int64_t> l2;
    std::vector<int64_t> l3;
    std::vector<int64_t> e2e;
    l1.reserve(kCount);
    l2.reserve(kCount);
    l3.reserve(kCount);
    e2e.reserve(kCount);

    const auto t_start = nowNs();

    std::thread producer([&]() {
        for (int i = 0; i < kCount; ++i) {
            Token t{};
            t.seq = static_cast<uint64_t>(i);
            t.t0 = nowNs();
            (void)q1.pushDropOldest(t);
        }
        prod_done.store(true, std::memory_order_release);
    });

    std::thread stage1([&]() {
        Token t{};
        while (!prod_done.load(std::memory_order_acquire) || q1.size() > 0U) {
            if (!q1.pop(t)) {
                std::this_thread::yield();
                continue;
            }
            syntheticWork(80);
            t.t1 = nowNs();
            (void)q2.pushDropOldest(t);
        }
        s1_done.store(true, std::memory_order_release);
    });

    std::thread stage2([&]() {
        Token t{};
        while (!s1_done.load(std::memory_order_acquire) || q2.size() > 0U) {
            if (!q2.pop(t)) {
                std::this_thread::yield();
                continue;
            }
            syntheticWork(120);
            t.t2 = nowNs();
            (void)q3.pushDropOldest(t);
        }
        s2_done.store(true, std::memory_order_release);
    });

    Token t{};
    while (!s2_done.load(std::memory_order_acquire) || q3.size() > 0U) {
        if (!q3.pop(t)) {
            std::this_thread::yield();
            continue;
        }
        syntheticWork(60);
        t.t3 = nowNs();
        const int64_t t4 = nowNs();
        l1.push_back(t.t1 - t.t0);
        l2.push_back(t.t2 - t.t1);
        l3.push_back(t.t3 - t.t2);
        e2e.push_back(t4 - t.t0);
    }

    producer.join();
    stage1.join();
    stage2.join();
    const auto t_end = nowNs();
    const double elapsed_s = static_cast<double>(t_end - t_start) / 1e9;
    const double fps = elapsed_s > 0.0 ? static_cast<double>(e2e.size()) / elapsed_s : 0.0;

    // Serial baseline with identical work profile.
    std::vector<int64_t> serial_e2e;
    serial_e2e.reserve(kCount);
    const auto s_start = nowNs();
    for (int i = 0; i < kCount; ++i) {
        const int64_t a = nowNs();
        syntheticWork(80);
        syntheticWork(120);
        syntheticWork(60);
        const int64_t b = nowNs();
        serial_e2e.push_back(b - a);
    }
    const auto s_end = nowNs();
    const double serial_elapsed_s = static_cast<double>(s_end - s_start) / 1e9;
    const double serial_fps = serial_elapsed_s > 0.0 ? static_cast<double>(kCount) / serial_elapsed_s : 0.0;

    std::cout << "benchmark synthetic_pipeline\n";
    std::cout << "samples " << e2e.size() << "\n";
    std::cout << "fps " << fps << "\n";
    std::cout << "stage1_ns_p50 " << pct(l1, 0.50) << "\n";
    std::cout << "stage1_ns_p95 " << pct(l1, 0.95) << "\n";
    std::cout << "stage1_ns_p99 " << pct(l1, 0.99) << "\n";
    std::cout << "stage2_ns_p50 " << pct(l2, 0.50) << "\n";
    std::cout << "stage2_ns_p95 " << pct(l2, 0.95) << "\n";
    std::cout << "stage2_ns_p99 " << pct(l2, 0.99) << "\n";
    std::cout << "stage3_ns_p50 " << pct(l3, 0.50) << "\n";
    std::cout << "stage3_ns_p95 " << pct(l3, 0.95) << "\n";
    std::cout << "stage3_ns_p99 " << pct(l3, 0.99) << "\n";
    std::cout << "e2e_ns_p50 " << pct(e2e, 0.50) << "\n";
    std::cout << "e2e_ns_p95 " << pct(e2e, 0.95) << "\n";
    std::cout << "e2e_ns_p99 " << pct(e2e, 0.99) << "\n";
    std::cout << "drops_q1 " << q1.dropCount() << "\n";
    std::cout << "drops_q2 " << q2.dropCount() << "\n";
    std::cout << "drops_q3 " << q3.dropCount() << "\n";
    std::cout << "serial_fps " << serial_fps << "\n";
    std::cout << "serial_e2e_ns_p50 " << pct(serial_e2e, 0.50) << "\n";
    std::cout << "serial_e2e_ns_p95 " << pct(serial_e2e, 0.95) << "\n";
    std::cout << "serial_e2e_ns_p99 " << pct(serial_e2e, 0.99) << "\n";
    return 0;
}
