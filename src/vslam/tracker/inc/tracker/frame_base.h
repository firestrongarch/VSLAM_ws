#pragma once
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <mutex>

namespace vslam {
class FrameBase {
public:
    FrameBase() = default;
    void Pose(const Sophus::SE3d &pose);
    Sophus::SE3d Pose();
    
protected:
    Sophus::SE3d pose_;
    std::mutex pose_mutex_;
};
}