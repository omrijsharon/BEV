#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

namespace bev::ipc {

enum class PushResult {
    Ok,
    DroppedOldest,
    Full,
};

template <typename T>
class SpscRing {
    static_assert(std::is_trivially_copyable<T>::value, "SpscRing requires trivially copyable type");

public:
    explicit SpscRing(std::size_t capacity_pow2)
        : capacity_(capacity_pow2), mask_(capacity_pow2 - 1), buffer_(capacity_pow2) {}

    bool valid() const { return capacity_ >= 2 && ((capacity_ & (capacity_ - 1)) == 0); }
    std::size_t capacity() const { return capacity_; }

    PushResult pushDropOldest(const T& value) {
        if (!valid()) {
            return PushResult::Full;
        }

        std::uint64_t head = head_.load(std::memory_order_relaxed);
        std::uint64_t tail = tail_.load(std::memory_order_acquire);

        bool dropped = false;
        if ((head - tail) >= capacity_) {
            // Drop oldest entry.
            tail_.store(tail + 1, std::memory_order_release);
            dropped = true;
            dropped_.fetch_add(1, std::memory_order_release);
        }

        buffer_[head & mask_] = value;
        head_.store(head + 1, std::memory_order_release);
        pushed_.fetch_add(1, std::memory_order_release);
        return dropped ? PushResult::DroppedOldest : PushResult::Ok;
    }

    bool pop(T& out) {
        if (!valid()) {
            return false;
        }
        std::uint64_t tail = tail_.load(std::memory_order_relaxed);
        const std::uint64_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return false;
        }

        out = buffer_[tail & mask_];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    bool peekLatest(T& out) const {
        if (!valid()) {
            return false;
        }
        const std::uint64_t head = head_.load(std::memory_order_acquire);
        const std::uint64_t tail = tail_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }
        out = buffer_[(head - 1) & mask_];
        return true;
    }

    std::size_t size() const {
        const std::uint64_t head = head_.load(std::memory_order_acquire);
        const std::uint64_t tail = tail_.load(std::memory_order_acquire);
        return static_cast<std::size_t>(head - tail);
    }

    std::uint64_t dropCount() const { return dropped_.load(std::memory_order_acquire); }
    std::uint64_t pushCount() const { return pushed_.load(std::memory_order_acquire); }

private:
    std::size_t capacity_{0};
    std::size_t mask_{0};
    std::vector<T> buffer_{};
    std::atomic<std::uint64_t> head_{0};
    std::atomic<std::uint64_t> tail_{0};
    std::atomic<std::uint64_t> pushed_{0};
    std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace bev::ipc
