#include "ipc/control_plane.hpp"

#ifdef __linux__
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace bev::ipc {

UnixControlServer::~UnixControlServer() {
    stop();
}

bool UnixControlServer::start(const std::string& socket_path, Handler handler, std::string& error) {
    stop();
#ifdef __linux__
    socket_path_ = socket_path;
    handler_ = std::move(handler);
    running_.store(true);
    thread_ = std::thread(&UnixControlServer::serveLoop, this);
    error.clear();
    return true;
#else
    (void)socket_path;
    (void)handler;
    error = "UnixControlServer requires Linux";
    return false;
#endif
}

void UnixControlServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }
#ifdef __linux__
    if (listen_fd_ >= 0) {
        shutdown(listen_fd_, SHUT_RDWR);
        close(listen_fd_);
        listen_fd_ = -1;
    }
#endif
    if (thread_.joinable()) {
        thread_.join();
    }
}

void UnixControlServer::serveLoop() {
#ifdef __linux__
    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        running_.store(false);
        return;
    }

    ::unlink(socket_path_.c_str());

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socket_path_.c_str());
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(listen_fd_);
        listen_fd_ = -1;
        running_.store(false);
        return;
    }
    if (listen(listen_fd_, 4) < 0) {
        close(listen_fd_);
        listen_fd_ = -1;
        running_.store(false);
        return;
    }

    while (running_.load()) {
        int client_fd = accept(listen_fd_, nullptr, nullptr);
        if (client_fd < 0) {
            if (running_.load()) {
                continue;
            }
            break;
        }

        char buf[512];
        const ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        std::string reply = "ERR empty\n";
        if (n > 0) {
            buf[n] = '\0';
            reply = handler_ ? handler_(std::string(buf)) : "ERR no-handler\n";
        }
        send(client_fd, reply.data(), reply.size(), 0);
        close(client_fd);
    }

    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }
    ::unlink(socket_path_.c_str());
#endif
}

bool unixControlRequest(const std::string& socket_path, const std::string& request, std::string& response, std::string& error) {
#ifdef __linux__
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        error = "socket failed";
        return false;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socket_path.c_str());
    if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        error = "connect failed";
        close(fd);
        return false;
    }
    if (send(fd, request.data(), request.size(), 0) < 0) {
        error = "send failed";
        close(fd);
        return false;
    }
    response.clear();
    char buf[2048];
    while (true) {
        const ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < 0) {
            error = "recv failed";
            close(fd);
            return false;
        }
        if (n == 0) {
            break;
        }
        response.append(buf, static_cast<std::size_t>(n));
    }
    close(fd);
    error.clear();
    return true;
#else
    (void)socket_path;
    (void)request;
    (void)response;
    error = "unixControlRequest requires Linux";
    return false;
#endif
}

}  // namespace bev::ipc
