#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "camera/camera_calibration.hpp"
#include "core/config.hpp"
#include "core/types.hpp"

namespace bev {

class CameraIngest {
public:
    CameraIngest() = default;

    bool openDevice(const CameraConfig& config, std::string& error);
    void close();
    bool isOpen() const;
    const std::string& activeSourceDescription() const { return active_source_desc_; }

    void setCalibration(const CameraCalibrationData& calibration);
    bool captureFrame(FramePacket& out_packet, std::string& error);

private:
    bool openV4L2(int device_index, std::string& error);
    bool openGStreamer(const std::string& pipeline, std::string& error);

    cv::VideoCapture cap_;
    CameraConfig config_{};
    CameraCalibrationData calibration_;
    bool has_calibration_{false};
    std::string active_source_desc_;
};

}  // namespace bev
