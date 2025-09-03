export module frame;

import cv;
import std;

export namespace Yslam {

struct MapPoint {
    cv::Point3d p3d; // 3D position
    cv::Point2d p2d; // 2D position in the image
};

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    std::vector<cv::Point2f> pts; // Extracted Points

    inline static std::shared_ptr<Frame> last;
};

} // namespace Yslam