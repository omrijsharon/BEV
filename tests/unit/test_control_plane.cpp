#include "ipc/control_plane.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#ifdef __linux__
#include <unistd.h>
#endif

int main() {
#ifdef __linux__
    const std::string sock = "/tmp/bev_test_ctl_" + std::to_string(static_cast<long long>(::getpid())) + ".sock";
    bev::ipc::UnixControlServer server;
    std::string err;
    if (!server.start(sock, [](const std::string& req) {
            if (req.find("ping") != std::string::npos) {
                return std::string("OK pong\n");
            }
            return std::string("ERR\n");
        }, err)) {
        std::cerr << "server.start failed: " << err << "\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::string resp;
    if (!bev::ipc::unixControlRequest(sock, "ping\n", resp, err)) {
        std::cerr << "request failed: " << err << "\n";
        return 1;
    }
    if (resp.find("OK pong") == std::string::npos) {
        std::cerr << "unexpected response\n";
        return 1;
    }
    server.stop();
    return 0;
#else
    return 0;
#endif
}
