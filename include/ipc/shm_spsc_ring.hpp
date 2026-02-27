#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace bev::ipc {

struct alignas(64) ShmSpscRingHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t elem_size;
    uint32_t capacity;
    std::atomic<uint64_t> head;
    std::atomic<uint64_t> tail;
    std::atomic<uint64_t> pushed;
    std::atomic<uint64_t> dropped;
    uint8_t reserved[24];
};

class ShmSpscRing {
public:
    ShmSpscRing() = default;
    ~ShmSpscRing();

    bool create(const std::string& name, uint32_t elem_size, uint32_t capacity_pow2, std::string& error);
    bool attach(const std::string& name, uint32_t elem_size, uint32_t capacity_pow2, std::string& error);
    void close();

    bool isOpen() const { return base_ != nullptr; }
    uint32_t capacity() const;
    uint32_t elemSize() const;

    bool pushDropOldest(const void* elem);
    bool pop(void* out_elem);
    bool peekLatest(void* out_elem) const;

    uint64_t dropCount() const;
    uint64_t pushCount() const;
    uint64_t depth() const;

private:
    bool mapInternal(int fd, std::size_t total_bytes, std::string& error);
    std::size_t totalMappedBytes() const;
    uint8_t* elementPtr(uint64_t idx) const;

    std::string name_{};
    int fd_{-1};
    bool owner_{false};
    uint8_t* base_{nullptr};
    ShmSpscRingHeader* header_{nullptr};
    uint8_t* data_{nullptr};
};

template <typename T>
class TypedShmSpscRing {
    static_assert(std::is_trivially_copyable<T>::value, "TypedShmSpscRing requires trivially copyable type");

public:
    bool create(const std::string& name, uint32_t capacity_pow2, std::string& error) {
        return ring_.create(name, static_cast<uint32_t>(sizeof(T)), capacity_pow2, error);
    }
    bool attach(const std::string& name, uint32_t capacity_pow2, std::string& error) {
        return ring_.attach(name, static_cast<uint32_t>(sizeof(T)), capacity_pow2, error);
    }
    void close() { ring_.close(); }
    bool isOpen() const { return ring_.isOpen(); }

    bool pushDropOldest(const T& value) { return ring_.pushDropOldest(&value); }
    bool pop(T& out) { return ring_.pop(&out); }
    bool peekLatest(T& out) const { return ring_.peekLatest(&out); }

    uint64_t dropCount() const { return ring_.dropCount(); }
    uint64_t pushCount() const { return ring_.pushCount(); }
    uint64_t depth() const { return ring_.depth(); }

private:
    ShmSpscRing ring_{};
};

}  // namespace bev::ipc
