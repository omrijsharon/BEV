#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

namespace bev::ipc {

enum class FrameSlotState : uint32_t {
    Free = 0,
    Writing = 1,
    Ready = 2,
};

struct alignas(64) FrameSlotHeader {
    std::atomic<uint32_t> generation;
    std::atomic<uint32_t> refcount;
    std::atomic<uint32_t> state;
    uint32_t width;
    uint32_t height;
    uint32_t type;
    uint32_t stride;
    int64_t timestamp_ns;
    uint8_t reserved[32];
};

class ShmFramePool {
public:
    ShmFramePool() = default;
    ~ShmFramePool();

    bool create(const std::string& name, std::size_t slot_count, std::size_t payload_bytes_per_slot, std::string& error);
    bool attach(const std::string& name, std::size_t slot_count, std::size_t payload_bytes_per_slot, std::string& error);
    void close();

    bool isOpen() const { return base_ != nullptr; }
    std::size_t slotCount() const { return slot_count_; }
    std::size_t payloadBytesPerSlot() const { return payload_bytes_per_slot_; }

    bool acquireWritableSlot(std::size_t& slot_idx);
    bool publishWrittenSlot(std::size_t slot_idx, uint32_t width, uint32_t height, uint32_t type, uint32_t stride, int64_t timestamp_ns, uint32_t& generation_out);

    bool acquireReadSlot(std::size_t slot_idx, uint32_t expected_generation);
    void releaseSlot(std::size_t slot_idx);

    uint8_t* payloadPtr(std::size_t slot_idx);
    const FrameSlotHeader* header(std::size_t slot_idx) const;

private:
    bool mapInternal(int fd, std::size_t total_bytes, std::string& error);
    std::size_t totalMappedBytes() const;

    std::string name_{};
    int fd_{-1};
    bool owner_{false};
    std::size_t slot_count_{0};
    std::size_t payload_bytes_per_slot_{0};
    uint8_t* base_{nullptr};
    FrameSlotHeader* headers_{nullptr};
    uint8_t* payload_base_{nullptr};
    std::atomic<uint64_t> next_candidate_{0};
};

}  // namespace bev::ipc