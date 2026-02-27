#include "ipc/notifier.hpp"

#include <iostream>
#include <string>

int main() {
    bev::ipc::RingNotifier notifier;
    std::string error;
    if (!notifier.open(error)) {
        std::cerr << "notifier open failed: " << error << "\n";
        return 1;
    }

    if (notifier.usingEventFd()) {
        if (!notifier.notify()) {
            std::cerr << "notify failed\n";
            return 1;
        }
        if (!notifier.wait(100)) {
            std::cerr << "wait failed with eventfd mode\n";
            return 1;
        }
    } else {
        // Fallback mode is polling/no-op, wait should just timeout/return false.
        if (notifier.wait(1)) {
            std::cerr << "wait should not succeed in fallback mode\n";
            return 1;
        }
    }

    notifier.close();
    return 0;
}
