#pragma once

#include <cstdint>
#include <string>

namespace bev::ipc {

class RingNotifier {
public:
    RingNotifier() = default;
    ~RingNotifier();

    bool open(std::string& error);
    void close();

    bool notify();
    bool wait(int timeout_ms);

    bool usingEventFd() const { return use_eventfd_; }

private:
    int fd_{-1};
    bool use_eventfd_{false};
};

}  // namespace bev::ipc