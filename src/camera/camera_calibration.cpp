#include "camera/camera_calibration.hpp"

#include <fstream>
#include <regex>
#include <sstream>
#include <vector>

#include <opencv2/core/persistence.hpp>

namespace bev {

namespace {

bool parseMatrixFromNode(const cv::FileNode& node, cv::Mat& out) {
    if (node.empty()) {
        return false;
    }

    // OpenCV matrix form (e.g. !!opencv-matrix)
    node >> out;
    if (!out.empty()) {
        out.convertTo(out, CV_64F);
        return true;
    }

    // Sequence-of-sequences form (plain YAML lists)
    if (node.type() != cv::FileNode::SEQ) {
        return false;
    }
    if (node.size() == 0) {
        return false;
    }

    const cv::FileNode first = node[0];
    if (first.type() == cv::FileNode::SEQ) {
        const int rows = static_cast<int>(node.size());
        const int cols = static_cast<int>(first.size());
        if (cols <= 0) {
            return false;
        }
        out = cv::Mat(rows, cols, CV_64F);
        for (int r = 0; r < rows; ++r) {
            const cv::FileNode row = node[r];
            if (row.type() != cv::FileNode::SEQ || static_cast<int>(row.size()) != cols) {
                return false;
            }
            for (int c = 0; c < cols; ++c) {
                double v = 0.0;
                row[c] >> v;
                out.at<double>(r, c) = v;
            }
        }
        return true;
    }

    // Flat sequence -> row vector
    const int n = static_cast<int>(node.size());
    out = cv::Mat(1, n, CV_64F);
    for (int i = 0; i < n; ++i) {
        double v = 0.0;
        node[i] >> v;
        out.at<double>(0, i) = v;
    }
    return true;
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::vector<double> extractNumbers(const std::string& line) {
    static const std::regex number_re(R"(([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?))");
    std::vector<double> out;
    auto begin = std::sregex_iterator(line.begin(), line.end(), number_re);
    auto end = std::sregex_iterator();
    for (auto it = begin; it != end; ++it) {
        out.push_back(std::stod(it->str()));
    }
    return out;
}

bool parsePlainYamlCalibration(const std::string& file_path, cv::Mat& K, cv::Mat& D) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
        return false;
    }

    enum class Section { None, Camera, Dist };
    Section section = Section::None;
    std::vector<double> cam_vals;
    std::vector<double> dist_vals;

    std::string line;
    while (std::getline(ifs, line)) {
        const std::string t = trim(line);
        if (t.empty() || t[0] == '#') {
            continue;
        }

        if (t.rfind("camera_matrix:", 0) == 0) {
            section = Section::Camera;
            continue;
        }
        if (t.rfind("dist_coeff:", 0) == 0) {
            section = Section::Dist;
            continue;
        }
        if (t.find(':') != std::string::npos && t[0] != '-' && t[0] != '[') {
            section = Section::None;
            continue;
        }

        const auto nums = extractNumbers(t);
        if (nums.empty()) {
            continue;
        }
        if (section == Section::Camera) {
            cam_vals.insert(cam_vals.end(), nums.begin(), nums.end());
        } else if (section == Section::Dist) {
            dist_vals.insert(dist_vals.end(), nums.begin(), nums.end());
        }
    }

    if (cam_vals.size() < 9 || dist_vals.size() < 4) {
        return false;
    }

    K = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        K.at<double>(i / 3, i % 3) = cam_vals[i];
    }
    D = cv::Mat(1, static_cast<int>(dist_vals.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(dist_vals.size()); ++i) {
        D.at<double>(0, i) = dist_vals[i];
    }
    return true;
}

}  // namespace

bool CameraCalibration::loadFromFile(const std::string& file_path, std::string& error) {
    data_.K.release();
    data_.D.release();

    bool parsed = false;
    try {
        const cv::FileStorage fs(file_path, cv::FileStorage::READ);
        if (fs.isOpened()) {
            // Support both formats:
            // 1) OpenCV style: K / D
            // 2) Plain YAML style: camera_matrix / dist_coeff (if FileStorage can parse it)
            const bool got_k = parseMatrixFromNode(fs["K"], data_.K) ||
                               parseMatrixFromNode(fs["camera_matrix"], data_.K);
            const bool got_d = parseMatrixFromNode(fs["D"], data_.D) ||
                               parseMatrixFromNode(fs["dist_coeff"], data_.D);
            parsed = got_k && got_d;
        }
    } catch (const cv::Exception&) {
        parsed = false;
    }

    // Fallback parser for plain YAML list format if FileStorage path failed.
    if (!parsed) {
        parsed = parsePlainYamlCalibration(file_path, data_.K, data_.D);
    }

    // Normalize distortion shape to 1xN
    if (!data_.D.empty() && data_.D.rows > 1 && data_.D.cols == 1) {
        data_.D = data_.D.t();
    }

    if (!parsed || !isValid()) {
        error = "calibration file is missing valid camera_matrix/dist_coeff (or K/D)";
        return false;
    }

    error.clear();
    return true;
}

bool CameraCalibration::isValid() const {
    if (data_.K.empty() || data_.D.empty()) {
        return false;
    }
    if (data_.K.rows != 3 || data_.K.cols != 3) {
        return false;
    }
    if (data_.D.total() < 4) {
        return false;
    }
    return true;
}

}  // namespace bev
