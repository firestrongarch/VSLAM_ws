#pragma once
#include <memory>
#include <opencv2/opencv.hpp>

namespace Yslam {

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    cv::Mat mask; // Mask for extracting features

    std::vector<cv::Point2f> pts0; // Extracted Points

    static std::shared_ptr<Frame> last;
};

inline std::shared_ptr<Frame> Frame::last = nullptr;

} // namespace Yslam