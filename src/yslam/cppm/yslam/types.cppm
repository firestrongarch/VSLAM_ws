export module types;

import cv;
import std;

export namespace Yslam {

class MapPoint : public cv::Point3d {
public:
    using Ptr = std::shared_ptr<MapPoint>;
    MapPoint() = delete;
    MapPoint(const cv::Point3d& p, const int& id)
        : cv::Point3d(p)
        , ID(id)
    {
    }

    const int ID;
    inline static int factory_id = 0;
    bool is_outlier = false;
};

class KeyPoint : public cv::KeyPoint {
public:
    using Ptr = std::shared_ptr<KeyPoint>;
    KeyPoint() = default;
    KeyPoint(const cv::KeyPoint& kp)
        : cv::KeyPoint(kp)
    {
    }

    std::weak_ptr<MapPoint> map_point;
    cv::Mat des;
    cv::Point2f match;

    bool is_outlier = false;
};

struct Frame {
    double t; // Timestamp in microseconds
    cv::Mat img0; // Left image
    cv::Mat img1; // Right image

    std::vector<cv::Point2f> pts; // Extracted Points
    std::vector<KeyPoint> kps; // Extracted KeyPoints

    inline static cv::Mat K; // Camera intrinsic matrix
    inline static std::shared_ptr<Frame> last;
};

} // namespace Yslam