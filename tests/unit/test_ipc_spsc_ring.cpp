#include "ipc/spsc_ring.hpp"

#include <iostream>

int main() {
    bev::ipc::SpscRing<int> invalid_ring(3);
    if (invalid_ring.valid()) {
        std::cerr << "non power-of-two ring must be invalid\n";
        return 1;
    }

    bev::ipc::SpscRing<int> ring(4);
    if (!ring.valid()) {
        std::cerr << "power-of-two ring must be valid\n";
        return 1;
    }

    for (int i = 0; i < 4; ++i) {
        const auto res = ring.pushDropOldest(i);
        if (res != bev::ipc::PushResult::Ok) {
            std::cerr << "initial fill should not drop\n";
            return 1;
        }
    }

    if (ring.size() != 4U) {
        std::cerr << "ring size after fill mismatch\n";
        return 1;
    }

    const auto drop_res = ring.pushDropOldest(4);
    if (drop_res != bev::ipc::PushResult::DroppedOldest) {
        std::cerr << "overflow push should drop oldest\n";
        return 1;
    }
    if (ring.dropCount() != 1U || ring.pushCount() != 5U) {
        std::cerr << "ring push/drop counters mismatch\n";
        return 1;
    }

    int latest = -1;
    if (!ring.peekLatest(latest) || latest != 4) {
        std::cerr << "peekLatest mismatch\n";
        return 1;
    }

    int v = -1;
    for (int expected = 1; expected <= 4; ++expected) {
        if (!ring.pop(v) || v != expected) {
            std::cerr << "pop sequence mismatch\n";
            return 1;
        }
    }

    if (ring.pop(v)) {
        std::cerr << "ring should be empty\n";
        return 1;
    }

    return 0;
}
