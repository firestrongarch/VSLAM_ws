#include "odom_impl.h"
#include "toml++/toml.h"
// #include "toml.hpp"
#include "kitti.h"
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

    for (const auto& frame : frame_paths) {
        // std::println("Frame timestamp: {}", std::get<0>(frame));
        // std::println("Left image path: {}", std::get<1>(frame));
        // std::println("Right image path: {}", std::get<2>(frame));
        cv::Mat left_image = cv::imread(std::get<1>(frame), cv::IMREAD_GRAYSCALE);
        cv::Mat right_image = cv::imread(std::get<2>(frame), cv::IMREAD_GRAYSCALE);

        track({ std::get<0>(frame), left_image, right_image, nullptr });

        cv::imshow("Left Image", left_image);
        cv::waitKey(10);
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

void Odom::OdomImpl::track(Frame frame)
{
    std::println("Tracking frame at timestamp: {}", frame.t);
    // Implementation for tracking the given frame
    extractor_->extract(frame);
}

}