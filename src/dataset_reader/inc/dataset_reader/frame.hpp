#pragma once
#include <chrono>

struct Frame {
    using Timestamp = std::chrono::microseconds;
    
    Timestamp timestamp;          // 时间戳
    // cv::Mat color_image;          // RGB图像
    // cv::Mat depth_image;          // 深度图像
    // std::vector<float> imu_data;  // IMU测量值
    // cv::Mat pose;                 // 位姿矩阵 (4x4)
    
    // 可扩展字段
    // std::unordered_map<std::string, cv::Mat> custom_data;
};