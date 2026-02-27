#include "ipc/notifier.hpp"

#ifdef __linux__
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#endif

namespace bev::ipc {

RingNotifier::~RingNotifier() {
    close();
}

bool RingNotifier::open(std::string& error) {
    close();
#ifdef __linux__
    fd_ = eventfd(0, EFD_NONBLOCK | EFD_SEMAPHORE);
    if (fd_ >= 0) {
        use_eventfd_ = true;
        error.clear();
        return true;
    }
#endif
    use_eventfd_ = false;
    error.clear();
    return true;
}

void RingNotifier::close() {
#ifdef __linux__
    if (fd_ >= 0) {
        ::close(fd_);
    }
#endif
    fd_ = -1;
    use_eventfd_ = false;
}

bool RingNotifier::notify() {
#ifdef __linux__
    if (use_eventfd_ && fd_ >= 0) {
        uint64_t one = 1;
        return write(fd_, &one, sizeof(one)) == sizeof(one);
    }
#endif
    return true;
}

bool RingNotifier::wait(int timeout_ms) {
#ifdef __linux__
    if (use_eventfd_ && fd_ >= 0) {
        struct pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        const int rc = poll(&pfd, 1, timeout_ms);
        if (rc <= 0) {
            return false;
        }
        uint64_t v = 0;
        (void)read(fd_, &v, sizeof(v));
        return true;
    }
#else
    (void)timeout_ms;
#endif
    return false;
}

}  // namespace bev::ipc