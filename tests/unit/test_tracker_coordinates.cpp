#include "tracking/template_tracker.hpp"

#include <iostream>

int main() {
    const cv::Rect crop(100, 50, 20, 20);
    const cv::Point2f local(3.0F, 4.0F);
    const cv::Point2f global = bev::localToGlobal(crop, local);

    if (global.x != 103.0F || global.y != 54.0F) {
        std::cerr << "localToGlobal failed\n";
        return 1;
    }

    return 0;
}