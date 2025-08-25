#pragma once
#include <memory>
#include <opencv2/opencv.hpp>

namespace Yslam {

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    std::vector<cv::KeyPoint> kps0; // Tracking Points
    std::vector<cv::KeyPoint> kps1; // Tracking Points

    cv::Mat desc0; // Descriptors for left image
    cv::Mat desc1; // Descriptors for right image

    std::unique_ptr<Frame> last;
};

} // namespace Yslam