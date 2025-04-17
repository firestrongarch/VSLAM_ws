#pragma once
#include <chrono>
#include <string>

namespace fsa {

struct Frame {
    using Timestamp = std::chrono::microseconds;
    
    double timestamp;          // 时间戳
    std::string left_image_path;
    std::string right_image_path;
    // cv::Mat color_image;          // RGB图像
    // cv::Mat depth_image;          // 深度图像
    // std::vector<float> imu_data;  // IMU测量值
    // cv::Mat pose;                 // 位姿矩阵 (4x4)
    
    // 可扩展字段
    // std::unordered_map<std::string, cv::Mat> custom_data;
};

}

