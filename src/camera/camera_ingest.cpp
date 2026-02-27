#include "camera/camera_ingest.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "core/time_utils.hpp"

#ifdef __linux__
#include <unistd.h>
#endif

namespace bev {

bool CameraIngest::openV4L2(int device_index, std::string& error) {
#ifdef __linux__
    const std::string dev = "/dev/video" + std::to_string(device_index);
    if (::access(dev.c_str(), F_OK) != 0) {
        error = "V4L2 device not found: " + dev;
        return false;
    }
#endif
    if (!cap_.open(device_index, cv::CAP_V4L2)) {
        error = "failed to open V4L2 camera device index " + std::to_string(device_index);
        return false;
    }
    active_source_desc_ = "v4l2:/dev/video" + std::to_string(device_index);
    error.clear();
    return true;
}

bool CameraIngest::openGStreamer(const std::string& pipeline, std::string& error) {
    if (!cap_.open(pipeline, cv::CAP_GSTREAMER)) {
        error = "failed to open GStreamer pipeline";
        return false;
    }
    active_source_desc_ = "gstreamer:" + pipeline;
    error.clear();
    return true;
}

bool CameraIngest::openDevice(const CameraConfig& config, std::string& error) {
    config_ = config;
    close();

    bool opened = false;
    std::string open_error;

    if (config_.source_mode == "gstreamer") {
        opened = openGStreamer(config_.gstreamer_pipeline, open_error);
        if (!opened) {
            // Requested GStreamer path failed; fallback to V4L2 for resilience.
            opened = openV4L2(config_.device_index, open_error);
        }
    } else {
        opened = openV4L2(config_.device_index, open_error);
    }

    if (!opened) {
        error = open_error;
        return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config_.width));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config_.height));
    cap_.set(cv::CAP_PROP_FPS, static_cast<double>(config_.fps));

    error.clear();
    return true;
}

void CameraIngest::close() {
    if (cap_.isOpened()) {
        cap_.release();
    }
    active_source_desc_.clear();
}

bool CameraIngest::isOpen() const {
    return cap_.isOpened();
}

void CameraIngest::setCalibration(const CameraCalibrationData& calibration) {
    calibration_ = calibration;
    has_calibration_ = !calibration_.K.empty() && !calibration_.D.empty();
}

bool CameraIngest::captureFrame(FramePacket& out_packet, std::string& error) {
    if (!cap_.isOpened()) {
        error = "camera is not open";
        return false;
    }

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        error = "failed to capture frame";
        return false;
    }

    out_packet.timestamp_ns = nowSteadyNs();
    out_packet.raw_bgr = frame;

    if (has_calibration_) {
        cv::undistort(out_packet.raw_bgr, out_packet.undistorted_bgr, calibration_.K, calibration_.D);
    } else {
        out_packet.undistorted_bgr = out_packet.raw_bgr;
    }

    cv::cvtColor(out_packet.undistorted_bgr, out_packet.bev_gray, cv::COLOR_BGR2GRAY);
    error.clear();
    return true;
}

}  // namespace bev
