#include "odom_impl.h"
#include "toml++/toml.h"
// #include "toml.hpp"
#include "yslam/kitti.h"
#include "yslam/tracker.h"
#include <opencv2/opencv.hpp>
#include <print>

namespace Yslam {

Odom::OdomImpl::OdomImpl() = default;

Odom::OdomImpl::~OdomImpl() = default;

void Odom::OdomImpl::run(std::string config_file)
{
    readConfig(config_file);
    // Implementation for running the odometry with the given config file

    auto frame_paths = loadKittiDataset(dataset_path_);

    extractor_->load("E:/CPP/VSLAM_ws/src/config_pkg/model");
    for (const auto& frame : frame_paths) {
        cv::Mat left_image = cv::imread(std::get<1>(frame), cv::IMREAD_GRAYSCALE);
        cv::Mat right_image = cv::imread(std::get<2>(frame), cv::IMREAD_GRAYSCALE);

        track(std::make_shared<Frame>(std::get<0>(frame), left_image, right_image));
    }
}

void Odom::OdomImpl::readConfig(const std::string& config_file)
{
    auto config = toml::parse_file(config_file);

    dataset_path_ = config["dataset"]["path"].value_or("unknown");
}

void Odom::OdomImpl::stop()
{
    std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

void Odom::OdomImpl::track(std::shared_ptr<Frame> frame)
{
    std::println("Tracking frame at timestamp: {}", frame->t);
    LkTracker Lk;

    extractor_->extract({ .img = frame->img0,
        .mask = frame->mask,
        .pts = frame->pts0,
        .kps = frame->kps0,
        .desc = frame->desc0 });
    // Implementation for tracking the given frame
    if (frame->last) {
        Lk.track({ .img0 = frame->last->desc0,
            .img1 = frame->desc0,
            .pts0 = frame->last->pts0,
            .pts1 = frame->pts0 });
    }
    if (frame->pts0.size() < 100) {
        extractor_->extract({ .img = frame->img0,
            .mask = frame->mask,
            .pts = frame->pts0,
            .kps = frame->kps0,
            .desc = frame->desc0 });
    }

    viewer_->view({ .img = frame->img0, .pts = frame->pts0 });

    frame->last = frame;
}

}