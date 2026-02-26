#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace bev {

class MJPEGServer {
public:
    MJPEGServer();
    ~MJPEGServer();

    bool start(uint16_t port);
    void stop();
    bool isRunning() const { return running_.load(); }

    void addRoute(const std::string& route);
    void updateFrame(const std::string& route, const cv::Mat& bgr_frame, int jpeg_quality = 80);
    std::vector<unsigned char> latestJpeg(const std::string& route) const;

    std::string multipartContentType() const;
    std::string makeMultipartFrame(const std::string& route) const;

private:
    void serveLoop();
    bool handleClient(int client_fd);
    static std::string extractRequestPath(const std::string& request);

    std::atomic<bool> running_{false};
    uint16_t port_{0};
    std::thread server_thread_;
    int listen_fd_{-1};

    mutable std::mutex mutex_;
    std::map<std::string, std::vector<unsigned char>> latest_jpeg_by_route_;
};

}  // namespace bev
