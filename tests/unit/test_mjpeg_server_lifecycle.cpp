#include "web/mjpeg_server.hpp"

#include <chrono>
#include <thread>

int main() {
    bev::MJPEGServer server;
    server.addRoute("/stream/main");
    server.start(18888);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    server.stop();
    return 0;
}
