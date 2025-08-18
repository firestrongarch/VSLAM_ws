//
// Created by weihao on 23-8-9.
//
#include "chrono"
#include "common/read_kitii_dataset.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "glog/stl_logging.h"
#include "ssvio/system.hpp"

DEFINE_string(config_yaml_path, "../config/kitti_00.yaml",
    "System config file path");
DEFINE_string(kitti_dataset_path,
    "E:/datasets/Kitti00-10/00",
    "kitti dataset path");

int main(int argc, char** argv)
{
    google::InitGoogleLogging("test_system");
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = 0; // INFO level is 0 in glog
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// load sequence frames
    std::vector<std::string> image_left_vec_path, image_right_vec_path;
    std::vector<double> vec_timestamp;
    common::LoadKittiImagesTimestamps(fLS::FLAGS_kitti_dataset_path,
        image_left_vec_path,
        image_right_vec_path,
        vec_timestamp);
    const size_t num_images = image_left_vec_path.size();
    LOG(INFO) << "Num Images: " << num_images;

    /// Init SLAM System
    ssvio::System system(fLS::FLAGS_config_yaml_path);

    for (int ni = 0; ni < num_images && !system.getViewUi()->ShouldQuit(); ni++) {
        LOG_IF(INFO, ni % 100 == 99)
            << "Has processed " << ni + 1 << " frames." << std::endl;
        cv::Mat img_left = cv::imread(image_left_vec_path[ni], cv::IMREAD_GRAYSCALE);
        cv::Mat img_right = cv::imread(image_right_vec_path[ni], cv::IMREAD_GRAYSCALE);
        double timestamp = vec_timestamp[ni];
        LOG_IF(FATAL, img_left.empty())
            << "Failed to load image at: " << image_left_vec_path[ni];

        system.RunStep(img_left, img_right, timestamp);
        // usleep(1e4);
    }
    system.getViewUi()->SaveTrajectoryAsTUM();
    while (!system.getViewUi()->ShouldQuit())
        ;
    return 0;
}