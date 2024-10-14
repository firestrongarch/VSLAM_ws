#include "tracker/tracker.h"

namespace vslam {

void Tracker::RunBinocular(const cv::Mat &left_image, const cv::Mat &right_iamge, const double timestamp)
{
    // current_frame_ = std::make_shared<Frame>(left_image, right_iamge, timestamp);

    // switch (track_status_)
    // {
    // case INIT:
    //     Init();
    //     break;
    // case GOOD:
    //     Track();
    //     break;
    // default:
    //     break;
    // }

    // ui_pangolin_->AddTrajectoryPose(current_frame_->Pose().inverse());
    // last_frame_ = current_frame_;
}

}