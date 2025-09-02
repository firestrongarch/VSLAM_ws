export module frame;

import cv;
import std;

export namespace Yslam {

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    std::vector<cv::Point2f> pts; // Extracted Points

    static std::shared_ptr<Frame> last;
};

inline std::shared_ptr<Frame> Frame::last = nullptr;

} // namespace Yslam