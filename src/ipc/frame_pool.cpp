#include "ipc/frame_pool.hpp"

#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace bev::ipc {

namespace {

inline std::size_t alignUp(std::size_t v, std::size_t align) {
    return (v + align - 1) & ~(align - 1);
}

}  // namespace

ShmFramePool::~ShmFramePool() {
    close();
}

bool ShmFramePool::mapInternal(int fd, std::size_t total_bytes, std::string& error) {
#ifdef __linux__
    void* ptr = mmap(nullptr, total_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        error = "mmap failed";
        return false;
    }

    base_ = static_cast<uint8_t*>(ptr);
    headers_ = reinterpret_cast<FrameSlotHeader*>(base_);
    payload_base_ = base_ + alignUp(sizeof(FrameSlotHeader) * slot_count_, 64);
    return true;
#else
    (void)fd;
    (void)total_bytes;
    error = "ShmFramePool requires Linux/POSIX shared memory";
    return false;
#endif
}

std::size_t ShmFramePool::totalMappedBytes() const {
    return alignUp(sizeof(FrameSlotHeader) * slot_count_, 64) + slot_count_ * payload_bytes_per_slot_;
}

bool ShmFramePool::create(const std::string& name, std::size_t slot_count, std::size_t payload_bytes_per_slot, std::string& error) {
    close();
    if (slot_count == 0 || payload_bytes_per_slot == 0) {
        error = "slot_count and payload_bytes_per_slot must be > 0";
        return false;
    }
#ifdef __linux__
    name_ = name;
    slot_count_ = slot_count;
    payload_bytes_per_slot_ = payload_bytes_per_slot;

    fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
        error = "shm_open create failed";
        return false;
    }
    owner_ = true;

    const std::size_t bytes = totalMappedBytes();
    if (ftruncate(fd_, static_cast<off_t>(bytes)) != 0) {
        error = "ftruncate failed";
        close();
        return false;
    }

    if (!mapInternal(fd_, bytes, error)) {
        close();
        return false;
    }

    std::memset(base_, 0, bytes);
    for (std::size_t i = 0; i < slot_count_; ++i) {
        headers_[i].generation.store(1, std::memory_order_relaxed);
        headers_[i].refcount.store(0, std::memory_order_relaxed);
        headers_[i].state.store(static_cast<uint32_t>(FrameSlotState::Free), std::memory_order_relaxed);
    }
    error.clear();
    return true;
#else
    (void)name;
    (void)slot_count;
    (void)payload_bytes_per_slot;
    error = "ShmFramePool requires Linux/POSIX shared memory";
    return false;
#endif
}

bool ShmFramePool::attach(const std::string& name, std::size_t slot_count, std::size_t payload_bytes_per_slot, std::string& error) {
    close();
#ifdef __linux__
    name_ = name;
    slot_count_ = slot_count;
    payload_bytes_per_slot_ = payload_bytes_per_slot;

    fd_ = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd_ < 0) {
        error = "shm_open attach failed";
        return false;
    }
    owner_ = false;

    const std::size_t bytes = totalMappedBytes();
    if (!mapInternal(fd_, bytes, error)) {
        close();
        return false;
    }

    error.clear();
    return true;
#else
    (void)name;
    (void)slot_count;
    (void)payload_bytes_per_slot;
    error = "ShmFramePool requires Linux/POSIX shared memory";
    return false;
#endif
}

void ShmFramePool::close() {
#ifdef __linux__
    if (base_) {
        munmap(base_, totalMappedBytes());
    }
    base_ = nullptr;
    headers_ = nullptr;
    payload_base_ = nullptr;

    if (fd_ >= 0) {
        ::close(fd_);
    }
    fd_ = -1;

#else
#endif
    owner_ = false;
    name_.clear();
    slot_count_ = 0;
    payload_bytes_per_slot_ = 0;
    next_candidate_.store(0, std::memory_order_relaxed);
}

bool ShmFramePool::acquireWritableSlot(std::size_t& slot_idx) {
    if (!isOpen()) {
        return false;
    }

    const std::size_t start = static_cast<std::size_t>(next_candidate_.fetch_add(1, std::memory_order_relaxed));
    for (std::size_t n = 0; n < slot_count_; ++n) {
        const std::size_t i = (start + n) % slot_count_;
        auto& h = headers_[i];
        if (h.state.load(std::memory_order_acquire) != static_cast<uint32_t>(FrameSlotState::Free)) {
            continue;
        }
        uint32_t expected = 0;
        if (!h.refcount.compare_exchange_strong(expected, 1, std::memory_order_acq_rel)) {
            continue;
        }
        h.state.store(static_cast<uint32_t>(FrameSlotState::Writing), std::memory_order_release);
        slot_idx = i;
        return true;
    }
    return false;
}

bool ShmFramePool::publishWrittenSlot(std::size_t slot_idx, uint32_t width, uint32_t height, uint32_t type, uint32_t stride, int64_t timestamp_ns, uint32_t& generation_out) {
    if (!isOpen() || slot_idx >= slot_count_) {
        return false;
    }

    auto& h = headers_[slot_idx];
    h.width = width;
    h.height = height;
    h.type = type;
    h.stride = stride;
    h.timestamp_ns = timestamp_ns;

    const uint32_t g = h.generation.fetch_add(1, std::memory_order_acq_rel) + 1;
    // Release producer ownership; readers now control slot liveness.
    h.refcount.store(0, std::memory_order_release);
    h.state.store(static_cast<uint32_t>(FrameSlotState::Ready), std::memory_order_release);
    generation_out = g;
    return true;
}

bool ShmFramePool::acquireReadSlot(std::size_t slot_idx, uint32_t expected_generation) {
    if (!isOpen() || slot_idx >= slot_count_) {
        return false;
    }

    auto& h = headers_[slot_idx];
    if (h.state.load(std::memory_order_acquire) != static_cast<uint32_t>(FrameSlotState::Ready)) {
        return false;
    }
    if (h.generation.load(std::memory_order_acquire) != expected_generation) {
        return false;
    }

    h.refcount.fetch_add(1, std::memory_order_acq_rel);
    if (h.generation.load(std::memory_order_acquire) != expected_generation) {
        h.refcount.fetch_sub(1, std::memory_order_acq_rel);
        return false;
    }
    return true;
}

void ShmFramePool::releaseSlot(std::size_t slot_idx) {
    if (!isOpen() || slot_idx >= slot_count_) {
        return;
    }
    auto& h = headers_[slot_idx];
    const uint32_t prev = h.refcount.fetch_sub(1, std::memory_order_acq_rel);
    if (prev == 1) {
        h.state.store(static_cast<uint32_t>(FrameSlotState::Free), std::memory_order_release);
    }
}

uint8_t* ShmFramePool::payloadPtr(std::size_t slot_idx) {
    if (!isOpen() || slot_idx >= slot_count_) {
        return nullptr;
    }
    return payload_base_ + slot_idx * payload_bytes_per_slot_;
}

const FrameSlotHeader* ShmFramePool::header(std::size_t slot_idx) const {
    if (!isOpen() || slot_idx >= slot_count_) {
        return nullptr;
    }
    return &headers_[slot_idx];
}

}  // namespace bev::ipc
