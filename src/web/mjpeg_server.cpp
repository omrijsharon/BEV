#include "web/mjpeg_server.hpp"

#include <chrono>
#include <cstring>

#include <opencv2/imgcodecs.hpp>

#ifdef __linux__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace bev {

MJPEGServer::MJPEGServer() = default;

MJPEGServer::~MJPEGServer() {
    stop();
}

bool MJPEGServer::start(uint16_t port) {
    if (running_.load()) {
        return false;
    }
    if (port == 0) {
        return false;
    }
    port_ = port;

#ifdef __linux__
    running_.store(true);
    server_thread_ = std::thread(&MJPEGServer::serveLoop, this);
    return true;
#else
    return false;
#endif
}

void MJPEGServer::stop() {
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

    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

void MJPEGServer::addRoute(const std::string& route) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_jpeg_by_route_.find(route) == latest_jpeg_by_route_.end()) {
        latest_jpeg_by_route_[route] = {};
    }
}

void MJPEGServer::updateFrame(const std::string& route, const cv::Mat& bgr_frame, int jpeg_quality) {
    if (bgr_frame.empty()) {
        return;
    }

    std::vector<int> params{
        cv::IMWRITE_JPEG_QUALITY, jpeg_quality
    };
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", bgr_frame, encoded, params)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_jpeg_by_route_[route] = std::move(encoded);
}

std::vector<unsigned char> MJPEGServer::latestJpeg(const std::string& route) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = latest_jpeg_by_route_.find(route);
    if (it == latest_jpeg_by_route_.end()) {
        return {};
    }
    return it->second;
}

std::string MJPEGServer::multipartContentType() const {
    return "multipart/x-mixed-replace; boundary=frame";
}

std::string MJPEGServer::makeMultipartFrame(const std::string& route) const {
    const auto jpeg = latestJpeg(route);
    if (jpeg.empty()) {
        return {};
    }

    std::string out;
    out.reserve(jpeg.size() + 128U);
    out += "--frame\r\n";
    out += "Content-Type: image/jpeg\r\n";
    out += "Content-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";
    out.append(reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
    out += "\r\n";
    return out;
}

#ifdef __linux__
std::string MJPEGServer::extractRequestPath(const std::string& request) {
    // Minimal parser for: GET /path?query HTTP/1.1
    const std::size_t get_pos = request.find("GET ");
    if (get_pos == std::string::npos) {
        return {};
    }
    const std::size_t path_start = get_pos + 4;
    const std::size_t path_end = request.find(' ', path_start);
    if (path_end == std::string::npos || path_end <= path_start) {
        return {};
    }
    std::string path = request.substr(path_start, path_end - path_start);
    const std::size_t query_pos = path.find('?');
    if (query_pos != std::string::npos) {
        path = path.substr(0, query_pos);
    }
    return path;
}

void MJPEGServer::serveLoop() {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        running_.store(false);
        return;
    }

    int yes = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port_);

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
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            if (running_.load()) {
                continue;
            }
            break;
        }
        handleClient(client_fd);
        close(client_fd);
    }

    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }
}

bool MJPEGServer::handleClient(int client_fd) {
    char req_buf[1024];
    const ssize_t n = recv(client_fd, req_buf, sizeof(req_buf) - 1, 0);
    if (n <= 0) {
        return false;
    }
    req_buf[n] = '\0';
    const std::string req(req_buf);
    const std::string path = extractRequestPath(req);
    if (path.empty()) {
        const std::string not_found = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
        send(client_fd, not_found.data(), not_found.size(), 0);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_jpeg_by_route_.find(path) == latest_jpeg_by_route_.end()) {
            const std::string not_found = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
            send(client_fd, not_found.data(), not_found.size(), 0);
            return false;
        }
    }

    const std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Type: " + multipartContentType() + "\r\n\r\n";
    if (send(client_fd, header.data(), header.size(), 0) < 0) {
        return false;
    }

    while (running_.load()) {
        const std::string frame = makeMultipartFrame(path);
        if (!frame.empty()) {
            if (send(client_fd, frame.data(), frame.size(), 0) < 0) {
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }

    return true;
}
#endif

}  // namespace bev
