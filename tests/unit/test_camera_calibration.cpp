#include "camera/camera_calibration.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

int main() {
    bev::CameraCalibration calibration;
    std::string error;

    const char* candidates[] = {
        "config/camera_calibration.yaml",
        "../config/camera_calibration.yaml"
    };

    bool loaded = false;
    for (const char* path : candidates) {
        if (calibration.loadFromFile(path, error)) {
            loaded = true;
            break;
        }
    }

    if (!loaded) {
        std::cerr << "failed to load calibration: " << error << "\n";
        return 1;
    }
    if (!calibration.isValid()) {
        std::cerr << "calibration should be valid\n";
        return 1;
    }

    // Backward compatibility: OpenCV K/D format should still parse.
    const std::string legacy_path = "build_test_legacy_calib.yaml";
    {
        std::ofstream ofs(legacy_path);
        ofs << "%YAML:1.0\n"
            << "K: !!opencv-matrix\n"
            << "   rows: 3\n"
            << "   cols: 3\n"
            << "   dt: d\n"
            << "   data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]\n"
            << "D: !!opencv-matrix\n"
            << "   rows: 1\n"
            << "   cols: 5\n"
            << "   dt: d\n"
            << "   data: [0.0, 0.0, 0.0, 0.0, 0.0]\n";
    }

    bev::CameraCalibration legacy_calib;
    if (!legacy_calib.loadFromFile(legacy_path, error) || !legacy_calib.isValid()) {
        std::cerr << "legacy K/D calibration should parse\n";
        return 1;
    }
    std::remove(legacy_path.c_str());

    return 0;
}
