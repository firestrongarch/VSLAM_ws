#include "tracker/frame_base.h"
#include <mutex>

namespace vslam {

void FrameBase::Pose(const Sophus::SE3d& pose) {
    std::lock_guard lock{ pose_mutex_ };
    pose_ = pose;
}

Sophus::SE3d FrameBase::Pose() {
    std::lock_guard lock{ pose_mutex_ };
    return pose_;
}

}