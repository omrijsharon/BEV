#include "ipc/frame_pool.hpp"

#include <cstring>
#include <iostream>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

int main() {
    bev::ipc::ShmFramePool pool;
    std::string error;

#ifdef __linux__
    const std::string name = "/bev_test_pool_" + std::to_string(static_cast<long long>(::getpid()));
    if (!pool.create(name, 4, 1024, error)) {
        std::cerr << "create failed: " << error << "\n";
        return 1;
    }

    for (int i = 0; i < 8; ++i) {
        std::size_t slot_idx = 0;
        if (!pool.acquireWritableSlot(slot_idx)) {
            std::cerr << "acquireWritableSlot failed at iteration " << i << "\n";
            return 1;
        }

        auto* payload = pool.payloadPtr(slot_idx);
        if (!payload) {
            std::cerr << "payloadPtr failed\n";
            return 1;
        }
        std::memset(payload, 0x5A, 64);

        uint32_t generation = 0;
        if (!pool.publishWrittenSlot(slot_idx, 640, 480, 16, 640, 123456789LL + i, generation)) {
            std::cerr << "publishWrittenSlot failed\n";
            return 1;
        }

        if (pool.acquireReadSlot(slot_idx, generation + 1)) {
            std::cerr << "generation mismatch must fail\n";
            return 1;
        }

        if (!pool.acquireReadSlot(slot_idx, generation)) {
            std::cerr << "acquireReadSlot failed\n";
            return 1;
        }

        const auto* hdr = pool.header(slot_idx);
        if (!hdr || hdr->width != 640U || hdr->height != 480U || hdr->timestamp_ns != (123456789LL + i)) {
            std::cerr << "header metadata mismatch\n";
            return 1;
        }

        pool.releaseSlot(slot_idx); // reader release
        if (hdr->state.load(std::memory_order_acquire) != static_cast<uint32_t>(bev::ipc::FrameSlotState::Free)) {
            std::cerr << "slot should be free after reader release\n";
            return 1;
        }
    }

    pool.close();
    return 0;
#else
    if (pool.create("/bev_test_pool_unsupported", 4, 1024, error)) {
        std::cerr << "create should fail on non-linux\n";
        return 1;
    }
    return 0;
#endif
}
