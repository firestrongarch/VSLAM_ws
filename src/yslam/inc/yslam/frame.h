#pragma once
#include <memory>
#include <opencv2/opencv.hpp>

namespace Yslam {

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    cv::Mat mask; // Mask for extracting features

    int cell_num = 20;
    std::vector<cv::KeyPoint> kps0; // Extracted Points
    std::vector<cv::KeyPoint> kps1; // Extracted Points
    std::vector<cv::Point2f> pts0; // Extracted Points
    std::vector<cv::Point2f> pts1; // Extracted Points
    std::vector<cv::Point2f> res; // Tracking result

    cv::Mat desc0; // Descriptors for left image
    cv::Mat desc1; // Descriptors for right image

    static std::shared_ptr<Frame> last;
};

inline std::shared_ptr<Frame> Frame::last = nullptr;

} // namespace Yslam