#include "ipc/descriptors.hpp"
#include "ipc/frame_pool.hpp"
#include "ipc/shm_spsc_ring.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core/hal/interface.h>

#ifdef __linux__
#include <unistd.h>
#endif

namespace {

bool runProducer(
    bev::ipc::ShmFramePool& pool,
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc>& ring,
    std::atomic<bool>& running,
    std::atomic<uint64_t>& seq_counter) {
    while (running.load(std::memory_order_acquire)) {
        std::size_t slot = 0;
        if (!pool.acquireWritableSlot(slot)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        uint8_t* dst = pool.payloadPtr(slot);
        if (dst == nullptr) {
            pool.releaseSlot(slot);
            continue;
        }

        std::memset(dst, 0x2A, 32 * 32);
        uint32_t gen = 0;
        const int64_t ts = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        bool published = false;
        if (pool.publishWrittenSlot(slot, 32U, 32U, static_cast<uint32_t>(CV_8UC1), 32U, ts, gen)) {
            auto d = bev::ipc::makeBevFrameDesc();
            d.frame_slot_id = static_cast<uint32_t>(slot);
            d.generation = gen;
            d.width = 32U;
            d.height = 32U;
            d.type = static_cast<uint32_t>(CV_8UC1);
            d.stride = 32U;
            d.timestamp_ns = ts;
            d.sequence_id = seq_counter.fetch_add(1, std::memory_order_acq_rel) + 1U;
            (void)ring.pushDropOldest(d);
            published = true;
        }
        if (!published) {
            pool.releaseSlot(slot);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

uint64_t runConsumerFor(
    const std::string& pool_name,
    const std::string& ring_name,
    int ms_duration,
    std::size_t slot_count,
    std::size_t payload_bytes) {
    bev::ipc::ShmFramePool pool;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> ring;
    std::string err;

    for (int i = 0; i < 100; ++i) {
        if (pool.attach(pool_name, slot_count, payload_bytes, err)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!pool.isOpen()) {
        return 0;
    }
    for (int i = 0; i < 100; ++i) {
        if (ring.attach(ring_name, 64U, err)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!ring.isOpen()) {
        return 0;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms_duration);
    uint64_t consumed = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        bev::ipc::BevFrameDesc d{};
        if (!ring.pop(d)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        const std::size_t slot = static_cast<std::size_t>(d.frame_slot_id);
        if (!pool.acquireReadSlot(slot, d.generation)) {
            continue;
        }
        const auto* hdr = pool.header(slot);
        if (hdr != nullptr && hdr->width == 32U && hdr->height == 32U) {
            consumed++;
        }
        pool.releaseSlot(slot);
    }
    return consumed;
}

}  // namespace

int main() {
#ifdef __linux__
    const std::string ns = "/bev_restart_test_" + std::to_string(static_cast<long long>(::getpid()));
    const std::string pool_name = ns + "_pool";
    const std::string ring_name = ns + "_ring";

    constexpr std::size_t kSlots = 8U;
    constexpr std::size_t kPayloadBytes = 4096U;

    std::string err;
    bev::ipc::ShmFramePool pool_a;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> ring_a;
    if (!pool_a.create(pool_name, kSlots, kPayloadBytes, err)) {
        std::cerr << "initial pool create failed: " << err << "\n";
        return 1;
    }
    if (!ring_a.create(ring_name, 64U, err)) {
        std::cerr << "initial ring create failed: " << err << "\n";
        return 1;
    }

    std::atomic<bool> run_a{true};
    std::atomic<uint64_t> seq{0};
    std::thread producer_a([&]() { (void)runProducer(pool_a, ring_a, run_a, seq); });

    const uint64_t consumed_before = runConsumerFor(pool_name, ring_name, 250, kSlots, kPayloadBytes);

    // Simulate producer process crash.
    run_a.store(false, std::memory_order_release);
    producer_a.join();
    pool_a.close();
    ring_a.close();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Simulate producer restart and rehydrate same named resources.
    bev::ipc::ShmFramePool pool_b;
    bev::ipc::TypedShmSpscRing<bev::ipc::BevFrameDesc> ring_b;
    if (!pool_b.create(pool_name, kSlots, kPayloadBytes, err)) {
        std::cerr << "restart pool create failed: " << err << "\n";
        return 1;
    }
    if (!ring_b.create(ring_name, 64U, err)) {
        std::cerr << "restart ring create failed: " << err << "\n";
        return 1;
    }
    std::atomic<bool> run_b{true};
    std::thread producer_b([&]() { (void)runProducer(pool_b, ring_b, run_b, seq); });

    const uint64_t consumed_after = runConsumerFor(pool_name, ring_name, 250, kSlots, kPayloadBytes);

    run_b.store(false, std::memory_order_release);
    producer_b.join();

    if (consumed_before == 0U || consumed_after == 0U) {
        std::cerr << "resilience test failed: consumed_before=" << consumed_before
                  << " consumed_after=" << consumed_after << "\n";
        return 1;
    }

    return 0;
#else
    return 0;
#endif
}
