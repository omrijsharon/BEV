#include "ipc/shm_spsc_ring.hpp"

#include <cstring>

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace bev::ipc {

namespace {

constexpr uint32_t kRingMagic = 0x42525652U;  // "BRVR"
constexpr uint32_t kRingVersion = 1U;

bool isPow2(uint32_t v) {
    return v > 1U && (v & (v - 1U)) == 0U;
}

std::size_t alignUp(std::size_t v, std::size_t align) {
    return (v + align - 1U) & ~(align - 1U);
}

}  // namespace

ShmSpscRing::~ShmSpscRing() {
    close();
}

bool ShmSpscRing::mapInternal(int fd, std::size_t total_bytes, std::string& error) {
#ifdef __linux__
    void* ptr = mmap(nullptr, total_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
        error = "mmap failed";
        return false;
    }
    base_ = static_cast<uint8_t*>(ptr);
    header_ = reinterpret_cast<ShmSpscRingHeader*>(base_);
    data_ = base_ + alignUp(sizeof(ShmSpscRingHeader), 64U);
    return true;
#else
    (void)fd;
    (void)total_bytes;
    error = "ShmSpscRing requires Linux/POSIX shared memory";
    return false;
#endif
}

std::size_t ShmSpscRing::totalMappedBytes() const {
    if (!header_) {
        return 0U;
    }
    return alignUp(sizeof(ShmSpscRingHeader), 64U) +
           static_cast<std::size_t>(header_->capacity) * header_->elem_size;
}

uint32_t ShmSpscRing::capacity() const {
    return header_ ? header_->capacity : 0U;
}

uint32_t ShmSpscRing::elemSize() const {
    return header_ ? header_->elem_size : 0U;
}

uint8_t* ShmSpscRing::elementPtr(uint64_t idx) const {
    const uint32_t cap = header_->capacity;
    const uint32_t mask = cap - 1U;
    return data_ + (static_cast<std::size_t>(idx & mask) * header_->elem_size);
}

bool ShmSpscRing::create(const std::string& name, uint32_t elem_size, uint32_t capacity_pow2, std::string& error) {
    close();
    if (elem_size == 0U || !isPow2(capacity_pow2)) {
        error = "invalid element size or capacity for shm ring";
        return false;
    }
#ifdef __linux__
    name_ = name;
    fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
        error = "shm_open create failed";
        return false;
    }
    owner_ = true;

    const std::size_t bytes = alignUp(sizeof(ShmSpscRingHeader), 64U) +
                              static_cast<std::size_t>(capacity_pow2) * elem_size;
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
    header_->magic = kRingMagic;
    header_->version = kRingVersion;
    header_->elem_size = elem_size;
    header_->capacity = capacity_pow2;
    header_->head.store(0, std::memory_order_relaxed);
    header_->tail.store(0, std::memory_order_relaxed);
    header_->pushed.store(0, std::memory_order_relaxed);
    header_->dropped.store(0, std::memory_order_relaxed);
    error.clear();
    return true;
#else
    (void)name;
    (void)elem_size;
    (void)capacity_pow2;
    error = "ShmSpscRing requires Linux/POSIX shared memory";
    return false;
#endif
}

bool ShmSpscRing::attach(const std::string& name, uint32_t elem_size, uint32_t capacity_pow2, std::string& error) {
    close();
#ifdef __linux__
    name_ = name;
    fd_ = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd_ < 0) {
        error = "shm_open attach failed";
        return false;
    }
    owner_ = false;

    const std::size_t bytes = alignUp(sizeof(ShmSpscRingHeader), 64U) +
                              static_cast<std::size_t>(capacity_pow2) * elem_size;
    if (!mapInternal(fd_, bytes, error)) {
        close();
        return false;
    }

    if (header_->magic != kRingMagic || header_->version != kRingVersion ||
        header_->elem_size != elem_size || header_->capacity != capacity_pow2) {
        error = "shm ring metadata mismatch";
        close();
        return false;
    }
    error.clear();
    return true;
#else
    (void)name;
    (void)elem_size;
    (void)capacity_pow2;
    error = "ShmSpscRing requires Linux/POSIX shared memory";
    return false;
#endif
}

void ShmSpscRing::close() {
#ifdef __linux__
    if (base_) {
        const std::size_t bytes = totalMappedBytes();
        if (bytes > 0U) {
            munmap(base_, bytes);
        }
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
#endif
    name_.clear();
    fd_ = -1;
    owner_ = false;
    base_ = nullptr;
    header_ = nullptr;
    data_ = nullptr;
}

bool ShmSpscRing::pushDropOldest(const void* elem) {
    if (!header_ || !elem) {
        return false;
    }
    uint64_t head = header_->head.load(std::memory_order_relaxed);
    uint64_t tail = header_->tail.load(std::memory_order_acquire);
    if ((head - tail) >= header_->capacity) {
        header_->tail.store(tail + 1U, std::memory_order_release);
        header_->dropped.fetch_add(1U, std::memory_order_release);
    }
    std::memcpy(elementPtr(head), elem, header_->elem_size);
    header_->head.store(head + 1U, std::memory_order_release);
    header_->pushed.fetch_add(1U, std::memory_order_release);
    return true;
}

bool ShmSpscRing::pop(void* out_elem) {
    if (!header_ || !out_elem) {
        return false;
    }
    const uint64_t tail = header_->tail.load(std::memory_order_relaxed);
    const uint64_t head = header_->head.load(std::memory_order_acquire);
    if (tail == head) {
        return false;
    }
    std::memcpy(out_elem, elementPtr(tail), header_->elem_size);
    header_->tail.store(tail + 1U, std::memory_order_release);
    return true;
}

bool ShmSpscRing::peekLatest(void* out_elem) const {
    if (!header_ || !out_elem) {
        return false;
    }
    const uint64_t head = header_->head.load(std::memory_order_acquire);
    const uint64_t tail = header_->tail.load(std::memory_order_acquire);
    if (head == tail) {
        return false;
    }
    std::memcpy(out_elem, elementPtr(head - 1U), header_->elem_size);
    return true;
}

uint64_t ShmSpscRing::dropCount() const {
    return header_ ? header_->dropped.load(std::memory_order_acquire) : 0U;
}

uint64_t ShmSpscRing::pushCount() const {
    return header_ ? header_->pushed.load(std::memory_order_acquire) : 0U;
}

uint64_t ShmSpscRing::depth() const {
    if (!header_) {
        return 0U;
    }
    const uint64_t head = header_->head.load(std::memory_order_acquire);
    const uint64_t tail = header_->tail.load(std::memory_order_acquire);
    return (head >= tail) ? (head - tail) : 0U;
}

}  // namespace bev::ipc
