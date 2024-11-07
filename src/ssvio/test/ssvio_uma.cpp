//
// Created by weihao on 23-8-9.
//
#include "ssvio/system.hpp"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "common/read_kitii_dataset.hpp"
#include "chrono"
#include "LetNet/letnet.h"

DEFINE_string(config_yaml_path, "/home/weihao/codespace/ssvio/config/kitti_00.yaml",
              "System config file path");
DEFINE_string(dataset_path,
              "/home/weihao/dataset/kitti/data_odometry_gray/dataset/sequences/00",
              "dataset path");
void LoadImages(const std::string &strPath, std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps);
int main(int argc, char **argv)
{
  google::InitGoogleLogging("ssvio_uma");
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = google::INFO;
  google::ParseCommandLineFlags(&argc, &argv, true);

  /// load sequence frames
  std::vector<std::string> image_left_vec_path, image_right_vec_path;
  std::vector<double> vec_timestamp;
  LoadImages(FLAGS_dataset_path, image_left_vec_path, image_right_vec_path, vec_timestamp);

  const size_t num_images = image_left_vec_path.size();
  LOG(INFO) << "Num Images: " << num_images;

  /// Init SLAM System
  ssvio::System system(fLS::FLAGS_config_yaml_path);

    cv::FileStorage fsSettings(
            FLAGS_config_yaml_path,                    // path_to_settings
            cv::FileStorage::READ);     // 以只读方式打开

    cv::Mat K_l, K_r, R_l, T_l, D_l, D_r;

    //相机内参
    fsSettings["cam0"]["K"] >> K_l;
    fsSettings["cam1"]["K"] >> K_r;
    fsSettings["cam0"]["R"] >> R_l;
    fsSettings["cam0"]["T"] >> T_l;

    //去畸变参数
    fsSettings["cam0"]["D"] >> D_l;
    fsSettings["cam1"]["D"] >> D_r;

    //存储四个映射矩阵
    cv::Mat M1l,M2l,M1r,M2r;
    const int IMG_W = 1024;
    const int IMG_H = 768;
    cv::Size imageSize(IMG_W, IMG_H);
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K_l, D_l, K_r, D_r,
        imageSize, R_l, T_l, R1, R2, P1, P2, Q);
    cv::fisheye::initUndistortRectifyMap(K_l, D_l, R1, P1,imageSize, CV_16SC2, M1l, M2l);
    cv::fisheye::initUndistortRectifyMap(K_r, D_r, R2, P2,imageSize, CV_16SC2, M1r, M2r);

  cv::Mat imLeft, imRight, imLeftRect, imRightRect;
  for (int ni = 0; ni < num_images && !system.getViewUi()->ShouldQuit(); ni++)
  {
    LOG_IF(INFO, ni % 100 == 99)
        << "Has processed " << ni + 1 << " frames." << std::endl;
    cv::Mat imLeft = cv::imread(image_left_vec_path[ni]);
    cv::Mat imRight = cv::imread(image_right_vec_path[ni]);
    double timestamp = vec_timestamp[ni];
    LOG_IF(FATAL, imLeft.empty())
        << "Failed to load image at: " << image_left_vec_path[ni];
    cv::remap(              //重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
        imLeft,             //输入图像
        imLeftRect,         //输出图像
        M1l,                //第一个映射矩阵表
        M2l,                //第二个映射矩阵
        cv::INTER_LINEAR);
    // 右目
    cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

    static LetNet net("/home/fu/Downloads/LET-NET-main/model");
    cv::Mat left(IMG_H, IMG_W, CV_8UC3),right(IMG_H, IMG_W, CV_8UC3);
    net.extract(imLeftRect, left);
    net.extract(imRightRect, right);
    cv::cvtColor(left, left, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right, right, cv::COLOR_BGR2GRAY);
    system.RunStep(left, right, timestamp);
    // system.RunStep(imLeftRect, imRightRect, timestamp);
    // usleep(1e4);
  }
  system.getViewUi()->SaveTrajectoryAsTUM();
  while (!system.getViewUi()->ShouldQuit())
    ;
  return 0;
}

void LoadImages(const std::string &strPath, std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps)
{
    std::ifstream data;
    data.open(strPath + "/cam0/data.csv");
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);

    std::string line;
    std::string name;
    std::getline(data, line);
    while(std::getline(data, line))
    {
        std::istringstream ss(line);
        if(std::getline(ss, name, ',')){
            vstrImageLeft.push_back(strPath + "/cam0/data/" + name + ".png");
            vstrImageRight.push_back(strPath + "/cam1/data/" + name + ".png");

            std::stringstream ss(name);
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);
        }
    }
}