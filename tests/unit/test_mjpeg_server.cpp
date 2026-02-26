#include "web/mjpeg_server.hpp"

#include <iostream>

#include <opencv2/core.hpp>

int main() {
    bev::MJPEGServer server;
    server.addRoute("/stream/main");
    cv::Mat img(32, 32, CV_8UC3, cv::Scalar(10, 20, 30));
    server.updateFrame("/stream/main", img, 75);

    const auto jpeg = server.latestJpeg("/stream/main");
    if (jpeg.empty()) {
        std::cerr << "jpeg should not be empty\n";
        return 1;
    }

    const std::string chunk = server.makeMultipartFrame("/stream/main");
    if (chunk.find("Content-Type: image/jpeg") == std::string::npos) {
        std::cerr << "multipart header missing\n";
        return 1;
    }

    return 0;
}
