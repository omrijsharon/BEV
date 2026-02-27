#include "ipc/spsc_ring.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main() {
    bev::ipc::SpscRing<int> ring(64);
    if (!ring.valid()) {
        std::cerr << "ring invalid\n";
        return 1;
    }

    bool running = true;
    int consumed = 0;
    std::thread producer([&]() {
        for (int i = 0; i < 50000; ++i) {
            (void)ring.pushDropOldest(i);
        }
        running = false;
    });

    int last = -1;
    while (running || ring.size() > 0U) {
        int v = -1;
        if (ring.pop(v)) {
            last = v;
            consumed++;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
    }
    producer.join();

    if (ring.dropCount() == 0U) {
        std::cerr << "expected drops under overload\n";
        return 1;
    }
    if (last < 49000) {
        std::cerr << "drop-old policy did not preserve recency (last=" << last << ")\n";
        return 1;
    }
    if (consumed <= 0) {
        std::cerr << "no items consumed\n";
        return 1;
    }
    return 0;
}
