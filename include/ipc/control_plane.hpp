#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace bev::ipc {

class UnixControlServer {
public:
    using Handler = std::function<std::string(const std::string&)>;

    UnixControlServer() = default;
    ~UnixControlServer();

    bool start(const std::string& socket_path, Handler handler, std::string& error);
    void stop();
    bool isRunning() const { return running_.load(); }

private:
    void serveLoop();

    std::atomic<bool> running_{false};
    std::string socket_path_{};
    Handler handler_{};
    std::thread thread_{};
    int listen_fd_{-1};
};

bool unixControlRequest(const std::string& socket_path, const std::string& request, std::string& response, std::string& error);

}  // namespace bev::ipc
