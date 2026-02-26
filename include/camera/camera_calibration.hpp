#pragma once

#include <string>

#include <opencv2/core.hpp>

namespace bev {

struct CameraCalibrationData {
    cv::Mat K;
    cv::Mat D;
};

class CameraCalibration {
public:
    bool loadFromFile(const std::string& file_path, std::string& error);
    bool isValid() const;

    const CameraCalibrationData& data() const { return data_; }

private:
    CameraCalibrationData data_;
};

}  // namespace bev
