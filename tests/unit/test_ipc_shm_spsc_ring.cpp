#include "ipc/shm_spsc_ring.hpp"

#include <iostream>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

int main() {
    std::string err;
#ifdef __linux__
    const std::string name = "/bev_test_shm_ring_" + std::to_string(static_cast<long long>(::getpid()));
    bev::ipc::TypedShmSpscRing<int> writer;
    bev::ipc::TypedShmSpscRing<int> reader;

    if (!writer.create(name, 4U, err)) {
        std::cerr << "writer.create failed: " << err << "\n";
        return 1;
    }
    if (!reader.attach(name, 4U, err)) {
        std::cerr << "reader.attach failed: " << err << "\n";
        return 1;
    }

    for (int i = 0; i < 4; ++i) {
        if (!writer.pushDropOldest(i)) {
            std::cerr << "push failed\n";
            return 1;
        }
    }
    if (!writer.pushDropOldest(4)) {
        std::cerr << "overflow push failed\n";
        return 1;
    }
    if (writer.dropCount() != 1U) {
        std::cerr << "drop counter mismatch\n";
        return 1;
    }

    int v = -1;
    for (int expected = 1; expected <= 4; ++expected) {
        if (!reader.pop(v) || v != expected) {
            std::cerr << "pop order mismatch\n";
            return 1;
        }
    }
    writer.close();
    reader.close();
    return 0;
#else
    bev::ipc::TypedShmSpscRing<int> writer;
    if (writer.create("/bev_test_ring_nonlinux", 4U, err)) {
        std::cerr << "non-linux create should fail\n";
        return 1;
    }
    return 0;
#endif
}
