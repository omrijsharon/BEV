#include "registration/registration_refiner.hpp"
#include "tracking/keypoint_initializer.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Mat makeBaseTexture(int w, int h) {
    cv::Mat img(h, w, CV_8UC1, cv::Scalar(20));
    cv::RNG rng(12345);
    for (int i = 0; i < 120; ++i) {
        cv::Point p1(rng.uniform(0, w), rng.uniform(0, h));
        cv::Point p2(rng.uniform(0, w), rng.uniform(0, h));
        cv::line(img, p1, p2, cv::Scalar(rng.uniform(60, 220)), 1, cv::LINE_AA);
    }
    for (int i = 0; i < 90; ++i) {
        cv::Point c(rng.uniform(0, w), rng.uniform(0, h));
        int r = rng.uniform(3, 12);
        cv::circle(img, c, r, cv::Scalar(rng.uniform(80, 240)), 1, cv::LINE_AA);
    }
    return img;
}

cv::Mat makeTransform(double tx, double ty, double yaw_deg, cv::Point2d c) {
    const double a = yaw_deg * CV_PI / 180.0;
    const double ca = std::cos(a);
    const double sa = std::sin(a);
    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    H.at<double>(0, 0) = ca;
    H.at<double>(0, 1) = -sa;
    H.at<double>(1, 0) = sa;
    H.at<double>(1, 1) = ca;
    H.at<double>(0, 2) = tx + c.x - ca * c.x + sa * c.y;
    H.at<double>(1, 2) = ty + c.y - sa * c.x - ca * c.y;
    return H;
}

double meanAbsDiff(const cv::Mat& a, const cv::Mat& b) {
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    return cv::mean(diff)[0];
}

}  // namespace

int main() {
    constexpr int kW = 320;
    constexpr int kH = 240;
    constexpr int kFrames = 14;
    const cv::Mat base = makeBaseTexture(kW, kH);
    std::vector<cv::Mat> frames;
    std::vector<cv::Mat> H_base_to_frame;
    frames.reserve(kFrames);
    H_base_to_frame.reserve(kFrames);

    for (int i = 0; i < kFrames; ++i) {
        const cv::Mat H = makeTransform(1.2 * i, 0.7 * i, 0.35 * i, cv::Point2d(kW / 2.0, kH / 2.0));
        cv::Mat warped;
        cv::warpPerspective(base, warped, H, base.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(15));
        frames.push_back(warped);
        H_base_to_frame.push_back(H);
    }

    bev::RefinerConfig cfg;
    cfg.ransac_reproj_threshold_px = 3.0;
    cfg.min_inliers = 20;
    cfg.min_inlier_ratio = 0.35;
    bev::RegistrationRefiner refiner(cfg);

    cv::RNG rng(2026);
    int success_pairs = 0;
    int estimated_pairs = 0;
    double sum_before = 0.0;
    double sum_after = 0.0;
    double sum_inlier_ratio = 0.0;
    double sum_rmse = 0.0;
    for (int i = 1; i < kFrames; ++i) {
        const cv::Mat& prev = frames[i - 1];
        const cv::Mat& curr = frames[i];
        const cv::Mat H_prev_to_curr = H_base_to_frame[i] * H_base_to_frame[i - 1].inv();

        const auto points = bev::KeypointInitializer::initializeShiTomasi(prev, 140, 0.01, 6.0);
        std::vector<cv::Point2f> src;
        std::vector<cv::Point2f> dst;
        src.reserve(points.size());
        dst.reserve(points.size());

        for (const auto& p : points) {
            std::vector<cv::Point2f> single{p};
            std::vector<cv::Point2f> projected;
            cv::perspectiveTransform(single, projected, H_prev_to_curr);
            cv::Point2f q = projected[0];

            if (q.x < 2.0F || q.y < 2.0F || q.x >= (kW - 2.0F) || q.y >= (kH - 2.0F)) {
                continue;
            }

            // Simulate tracked correspondences from a recorded sequence:
            // small gaussian noise + occasional outliers.
            q.x += static_cast<float>(rng.gaussian(0.6));
            q.y += static_cast<float>(rng.gaussian(0.6));
            if (rng.uniform(0.0, 1.0) < 0.12) {
                q.x = static_cast<float>(rng.uniform(0, kW));
                q.y = static_cast<float>(rng.uniform(0, kH));
            }

            src.push_back(p);
            dst.push_back(q);
        }

        bev::RegistrationResult r;
        if (src.size() < 30 || !refiner.estimateHomography(src, dst, r)) {
            continue;
        }
        ++estimated_pairs;
        sum_inlier_ratio += r.inlier_ratio;
        sum_rmse += r.reproj_rmse;

        cv::Mat aligned;
        cv::warpPerspective(curr, aligned, r.H_refine.inv(), prev.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(15));

        const double before = meanAbsDiff(prev, curr);
        const double after = meanAbsDiff(prev, aligned);
        sum_before += before;
        sum_after += after;
        if (after < before * 0.85) {
            ++success_pairs;
        }
    }

    const double avg_before = (estimated_pairs > 0) ? (sum_before / estimated_pairs) : 0.0;
    const double avg_after = (estimated_pairs > 0) ? (sum_after / estimated_pairs) : 0.0;
    const double avg_inlier_ratio = (estimated_pairs > 0) ? (sum_inlier_ratio / estimated_pairs) : 0.0;
    const double avg_rmse = (estimated_pairs > 0) ? (sum_rmse / estimated_pairs) : 0.0;

    std::cout << std::fixed << std::setprecision(4)
              << "sequence_registration: estimated_pairs=" << estimated_pairs
              << " success_pairs=" << success_pairs
              << " avg_before=" << avg_before
              << " avg_after=" << avg_after
              << " avg_inlier_ratio=" << avg_inlier_ratio
              << " avg_rmse=" << avg_rmse << "\n";

    {
        const char* metric_paths[] = {
            "data/logs/metrics_sequence_registration.txt",
            "../data/logs/metrics_sequence_registration.txt"
        };
        for (const char* p : metric_paths) {
            std::ofstream ofs(p);
            if (!ofs.is_open()) {
                continue;
            }
            ofs << std::fixed << std::setprecision(6)
                << "estimated_pairs=" << estimated_pairs << "\n"
                << "success_pairs=" << success_pairs << "\n"
                << "avg_before=" << avg_before << "\n"
                << "avg_after=" << avg_after << "\n"
                << "avg_inlier_ratio=" << avg_inlier_ratio << "\n"
                << "avg_rmse=" << avg_rmse << "\n";
            break;
        }
    }

    if (success_pairs < 10) {
        std::cerr << "insufficient successful pair registrations: " << success_pairs << "\n";
        return 1;
    }
    if (avg_inlier_ratio < 0.55) {
        std::cerr << "avg inlier ratio too low: " << avg_inlier_ratio << "\n";
        return 1;
    }
    if (avg_after >= avg_before * 0.85) {
        std::cerr << "alignment improvement insufficient: before=" << avg_before << " after=" << avg_after << "\n";
        return 1;
    }

    return 0;
}
