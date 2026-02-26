#include "core/config.hpp"

#include <fstream>
#include <sstream>
#include <unordered_map>

#include <opencv2/core.hpp>

namespace bev {

namespace {

template <typename T>
void readOrDefault(const cv::FileNode& node, const char* key, T& out) {
    const cv::FileNode child = node[key];
    if (!child.empty()) {
        child >> out;
    }
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string unquote(std::string v) {
    v = trim(v);
    if (v.size() >= 2) {
        if ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\'')) {
            v = v.substr(1, v.size() - 2);
        }
    }
    return v;
}

bool toBool(const std::string& v, bool& out) {
    const std::string t = trim(v);
    if (t == "true" || t == "True" || t == "1") {
        out = true;
        return true;
    }
    if (t == "false" || t == "False" || t == "0") {
        out = false;
        return true;
    }
    return false;
}

bool loadConfigPlainYaml(const std::string& path, AppConfig& out, std::string& error) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        error = "failed to open config file: " + path;
        return false;
    }

    std::string section;
    std::string line;
    while (std::getline(ifs, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') {
            continue;
        }

        // section header, e.g. "camera:"
        if (!t.empty() && t.back() == ':' && t.find(' ') == std::string::npos) {
            section = t.substr(0, t.size() - 1);
            continue;
        }

        const auto colon = t.find(':');
        if (colon == std::string::npos || section.empty()) {
            continue;
        }

        const std::string key = trim(t.substr(0, colon));
        std::string value = trim(t.substr(colon + 1));
        // strip inline comment
        const auto hash = value.find(" #");
        if (hash != std::string::npos) {
            value = trim(value.substr(0, hash));
        }
        value = unquote(value);

        try {
            if (section == "camera") {
                if (key == "width") out.camera.width = std::stoi(value);
                else if (key == "height") out.camera.height = std::stoi(value);
                else if (key == "fps") out.camera.fps = std::stoi(value);
                else if (key == "source_mode") out.camera.source_mode = value;
                else if (key == "device_index") out.camera.device_index = std::stoi(value);
                else if (key == "gstreamer_pipeline") out.camera.gstreamer_pipeline = value;
            } else if (section == "camera_mount") {
                if (key == "roll_deg") out.camera_mount.roll_deg = std::stod(value);
                else if (key == "pitch_deg") out.camera_mount.pitch_deg = std::stod(value);
                else if (key == "yaw_deg") out.camera_mount.yaw_deg = std::stod(value);
                else if (key == "negate_yaw") {
                    bool b = out.camera_mount.negate_yaw;
                    if (toBool(value, b)) out.camera_mount.negate_yaw = b;
                } else if (key == "enable_roll_sign_ab_test") {
                    bool b = out.camera_mount.enable_roll_sign_ab_test;
                    if (toBool(value, b)) out.camera_mount.enable_roll_sign_ab_test = b;
                } else if (key == "ab_test_window_frames") out.camera_mount.ab_test_window_frames = std::stoi(value);
            } else if (section == "tracking") {
                if (key == "num_keypoints") out.tracking.num_keypoints = std::stoi(value);
                else if (key == "template_size") out.tracking.template_size = std::stoi(value);
                else if (key == "coarse_crop_size") out.tracking.coarse_crop_size = std::stoi(value);
                else if (key == "fine_crop_size") out.tracking.fine_crop_size = std::stoi(value);
                else if (key == "max_distance_from_keypoint_center") out.tracking.max_distance_from_keypoint_center = std::stof(value);
                else if (key == "min_valid_tracks") out.tracking.min_valid_tracks = std::stoi(value);
                else if (key == "ransac_reproj_threshold_px") out.tracking.ransac_reproj_threshold_px = std::stod(value);
            } else if (section == "motion") {
                if (key == "alpha") out.motion.alpha = std::stof(value);
                else if (key == "threshold") out.motion.threshold = std::stoi(value);
            } else if (section == "map") {
                if (key == "initial_width") out.map.initial_width = std::stoi(value);
                else if (key == "initial_height") out.map.initial_height = std::stoi(value);
                else if (key == "expand_margin_px") out.map.expand_margin_px = std::stoi(value);
            }
        } catch (const std::exception&) {
            // keep defaults/previous values on parse failure
        }
    }

    return validateConfig(out, error);
}

}  // namespace

bool validateConfig(const AppConfig& cfg, std::string& error) {
    if (cfg.camera.width <= 0 || cfg.camera.height <= 0 || cfg.camera.fps <= 0) {
        error = "camera dimensions/fps must be > 0";
        return false;
    }
    if (cfg.camera.source_mode != "v4l2" && cfg.camera.source_mode != "gstreamer") {
        error = "camera.source_mode must be 'v4l2' or 'gstreamer'";
        return false;
    }
    if (cfg.camera.source_mode == "gstreamer" && cfg.camera.gstreamer_pipeline.empty()) {
        error = "camera.gstreamer_pipeline must not be empty when source_mode=gstreamer";
        return false;
    }
    if (cfg.tracking.num_keypoints <= 0) {
        error = "tracking.num_keypoints must be > 0";
        return false;
    }
    if (cfg.tracking.template_size <= 2 || (cfg.tracking.template_size % 2) == 0) {
        error = "tracking.template_size must be odd and > 2";
        return false;
    }
    if (cfg.tracking.coarse_crop_size < cfg.tracking.template_size ||
        cfg.tracking.fine_crop_size < cfg.tracking.template_size) {
        error = "crop sizes must be >= template_size";
        return false;
    }
    if (cfg.tracking.max_distance_from_keypoint_center <= 0.0F) {
        error = "tracking.max_distance_from_keypoint_center must be > 0";
        return false;
    }
    auto inAngleRange = [](double d) { return d >= -180.0 && d <= 180.0; };
    if (!inAngleRange(cfg.camera_mount.roll_deg) ||
        !inAngleRange(cfg.camera_mount.pitch_deg) ||
        !inAngleRange(cfg.camera_mount.yaw_deg)) {
        error = "camera_mount angles must be in [-180, 180] degrees";
        return false;
    }
    if (cfg.camera_mount.ab_test_window_frames <= 0) {
        error = "camera_mount.ab_test_window_frames must be > 0";
        return false;
    }
    if (cfg.motion.alpha < 0.0F || cfg.motion.alpha > 1.0F) {
        error = "motion.alpha must be in [0,1]";
        return false;
    }
    if (cfg.motion.threshold < 0 || cfg.motion.threshold > 255) {
        error = "motion.threshold must be in [0,255]";
        return false;
    }
    if (cfg.map.initial_width <= 0 || cfg.map.initial_height <= 0 || cfg.map.expand_margin_px < 0) {
        error = "map dimensions must be > 0 and margin must be >= 0";
        return false;
    }
    error.clear();
    return true;
}

bool loadConfig(const std::string& path, AppConfig& out, std::string& error) {
    try {
        const cv::FileStorage fs(path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            const cv::FileNode camera = fs["camera"];
            const cv::FileNode camera_mount = fs["camera_mount"];
            const cv::FileNode tracking = fs["tracking"];
            const cv::FileNode motion = fs["motion"];
            const cv::FileNode map = fs["map"];

            readOrDefault(camera, "width", out.camera.width);
            readOrDefault(camera, "height", out.camera.height);
            readOrDefault(camera, "fps", out.camera.fps);
            readOrDefault(camera, "source_mode", out.camera.source_mode);
            readOrDefault(camera, "device_index", out.camera.device_index);
            readOrDefault(camera, "gstreamer_pipeline", out.camera.gstreamer_pipeline);

            readOrDefault(camera_mount, "roll_deg", out.camera_mount.roll_deg);
            readOrDefault(camera_mount, "pitch_deg", out.camera_mount.pitch_deg);
            readOrDefault(camera_mount, "yaw_deg", out.camera_mount.yaw_deg);
            readOrDefault(camera_mount, "negate_yaw", out.camera_mount.negate_yaw);
            readOrDefault(camera_mount, "enable_roll_sign_ab_test", out.camera_mount.enable_roll_sign_ab_test);
            readOrDefault(camera_mount, "ab_test_window_frames", out.camera_mount.ab_test_window_frames);

            readOrDefault(tracking, "num_keypoints", out.tracking.num_keypoints);
            readOrDefault(tracking, "template_size", out.tracking.template_size);
            readOrDefault(tracking, "coarse_crop_size", out.tracking.coarse_crop_size);
            readOrDefault(tracking, "fine_crop_size", out.tracking.fine_crop_size);
            readOrDefault(tracking, "max_distance_from_keypoint_center", out.tracking.max_distance_from_keypoint_center);
            readOrDefault(tracking, "min_valid_tracks", out.tracking.min_valid_tracks);
            readOrDefault(tracking, "ransac_reproj_threshold_px", out.tracking.ransac_reproj_threshold_px);

            readOrDefault(motion, "alpha", out.motion.alpha);
            readOrDefault(motion, "threshold", out.motion.threshold);

            readOrDefault(map, "initial_width", out.map.initial_width);
            readOrDefault(map, "initial_height", out.map.initial_height);
            readOrDefault(map, "expand_margin_px", out.map.expand_margin_px);

            return validateConfig(out, error);
        }
    } catch (const cv::Exception&) {
        // fall through to plain YAML parser below
    }

    return loadConfigPlainYaml(path, out, error);
}

}  // namespace bev
