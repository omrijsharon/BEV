#include "motion/motion_detector.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    constexpr int w = 220;
    constexpr int h = 160;

    cv::Mat base(h, w, CV_8UC1, cv::Scalar(80));
    for (int y = 0; y < h; y += 8) {
        cv::line(base, cv::Point(0, y), cv::Point(w - 1, y), cv::Scalar(90), 1);
    }

    cv::Mat f1 = base.clone();
    cv::Mat f2 = base.clone();

    cv::Rect obj1(70, 60, 24, 24);
    cv::Rect obj2(84, 60, 24, 24);
    cv::rectangle(f1, obj1, cv::Scalar(180), cv::FILLED);
    cv::rectangle(f2, obj2, cv::Scalar(180), cv::FILLED);

    cv::Mat diff;
    bev::computeSignedDiff(f1, f2, diff);
    bev::applyMotionDeadzoneInPlace(diff, 20);

    int non_zero_union = 0;
    int non_zero_outside = 0;
    cv::Rect union_rect = obj1 | obj2;
    int total_union = union_rect.width * union_rect.height;
    int total_outside = w * h - total_union;
    for (int y = 0; y < h; ++y) {
        const auto* row = diff.ptr<int16_t>(y);
        for (int x = 0; x < w; ++x) {
            if (row[x] == 0) {
                continue;
            }
            if (union_rect.contains(cv::Point(x, y))) {
                ++non_zero_union;
            } else {
                ++non_zero_outside;
            }
        }
    }

    const double tp_rate = (total_union > 0) ? static_cast<double>(non_zero_union) / static_cast<double>(total_union) : 0.0;
    const double fp_rate = (total_outside > 0) ? static_cast<double>(non_zero_outside) / static_cast<double>(total_outside) : 0.0;

    std::cout << std::fixed << std::setprecision(6)
              << "motion_replay: tp_rate=" << tp_rate
              << " fp_rate=" << fp_rate
              << " non_zero_union=" << non_zero_union
              << " non_zero_outside=" << non_zero_outside << "\n";

    {
        const char* metric_paths[] = {
            "data/logs/metrics_motion_replay.txt",
            "../data/logs/metrics_motion_replay.txt"
        };
        for (const char* p : metric_paths) {
            std::ofstream ofs(p);
            if (!ofs.is_open()) {
                continue;
            }
            ofs << std::fixed << std::setprecision(6)
                << "tp_rate=" << tp_rate << "\n"
                << "fp_rate=" << fp_rate << "\n"
                << "non_zero_union=" << non_zero_union << "\n"
                << "non_zero_outside=" << non_zero_outside << "\n";
            break;
        }
    }

    if (non_zero_union < 200) {
        std::cerr << "motion response in union region too low\n";
        return 1;
    }
    if (non_zero_outside > 40) {
        std::cerr << "too many false positives outside motion region\n";
        return 1;
    }
    if (fp_rate > 0.002) {
        std::cerr << "false-positive rate too high: " << fp_rate << "\n";
        return 1;
    }

    cv::Mat bgr;
    cv::cvtColor(f2, bgr, cv::COLOR_GRAY2BGR);
    cv::Mat overlay;
    bev::renderMotionOverlay(bgr, diff, 0.6F, overlay);
    if (overlay.empty()) {
        std::cerr << "overlay generation failed\n";
        return 1;
    }

    return 0;
}
