#include "ipc/shm_spsc_ring.hpp"

#include <iostream>
#include <string>

#ifdef __linux__
#include <unistd.h>
#endif

int main() {
    std::string err;
#ifdef __linux__
    const std::string name = "/bev_test_ring_mismatch_" + std::to_string(static_cast<long long>(::getpid()));
    bev::ipc::TypedShmSpscRing<int> writer;
    if (!writer.create(name, 64U, err)) {
        std::cerr << "writer.create failed: " << err << "\n";
        return 1;
    }

    bev::ipc::TypedShmSpscRing<int> bad_attach;
    if (bad_attach.attach(name, 128U, err)) {
        std::cerr << "attach with wrong capacity should fail\n";
        return 1;
    }
    if (err.find("mismatch") == std::string::npos) {
        std::cerr << "expected mismatch error text\n";
        return 1;
    }
    return 0;
#else
    return 0;
#endif
}
