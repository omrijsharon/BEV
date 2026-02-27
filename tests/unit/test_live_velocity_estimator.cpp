#include "tracking/template_tracker.hpp"
#include "tracking/track_manager.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    cv::Mat prev(140, 180, CV_8UC1, cv::Scalar(10));
    cv::Mat curr(140, 180, CV_8UC1, cv::Scalar(10));
    cv::Rect patch_rect_prev(70, 60, 21, 21);
    cv::Mat patch(21, 21, CV_8UC1, cv::Scalar(18));
    cv::rectangle(patch, cv::Rect(1, 2, 8, 6), cv::Scalar(245), cv::FILLED);
    cv::line(patch, cv::Point(2, 19), cv::Point(20, 11), cv::Scalar(200), 2);
    cv::circle(patch, cv::Point(15, 6), 3, cv::Scalar(220), cv::FILLED);
    patch.copyTo(prev(patch_rect_prev));

    cv::Rect patch_rect_curr = patch_rect_prev + cv::Point(6, -4);
    patch.copyTo(curr(patch_rect_curr));

    bev::TrackPoint tr;
    tr.id = 1;
    tr.pos = cv::Point2f(
        static_cast<float>(patch_rect_prev.x + patch_rect_prev.width / 2),
        static_cast<float>(patch_rect_prev.y + patch_rect_prev.height / 2));
    tr.vel_px_s = cv::Point2f(120.0F, -80.0F); // dt=0.05s predicts exactly (6,-4)
    tr.valid = true;
    tr.confidence = 1.0F;

    bev::TrackManager mgr(30.0F);
    const float dt_s = 0.05F;
    const cv::Point2f predicted = mgr.predictSearchCenter(tr, dt_s);
    const int coarse_size = 80;
    const int half = coarse_size / 2;
    const cv::Rect coarse_window(
        static_cast<int>(std::lround(predicted.x)) - half,
        static_cast<int>(std::lround(predicted.y)) - half,
        coarse_size,
        coarse_size);

    const cv::Rect templ_rect(
        static_cast<int>(std::lround(tr.pos.x)) - 10,
        static_cast<int>(std::lround(tr.pos.y)) - 10,
        21,
        21);
    const cv::Mat templ = prev(templ_rect);
    const auto m = bev::coarseFineMatch(curr, templ, coarse_window, 80, 2);
    if (!m.valid) {
        std::cerr << "match invalid/low score\n";
        return 1;
    }

    const cv::Point2f pos_new(
        m.global_top_left.x + 10.0F,
        m.global_top_left.y + 10.0F);
    const cv::Point2f vel(
        (pos_new.x - tr.pos.x) / dt_s,
        (pos_new.y - tr.pos.y) / dt_s);
    if (std::abs(vel.x - 120.0F) > 3.0F || std::abs(vel.y + 80.0F) > 3.0F) {
        std::cerr << "velocity mismatch vx=" << vel.x << " vy=" << vel.y << "\n";
        return 1;
    }

    return 0;
}
