#pragma once
#include <opencv2/opencv.hpp>
#include "net.h"

class LetNet{
public:
    LetNet() = default;
    LetNet(const std::string& path);
    // bool detect(cv::Mat& img, std::vector<float>& result);
public:
    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1.0/255.0, 1.0/255.0, 1.0/255.0};
    const float mean_vals_inv[3] = {0, 0, 0};
    const float norm_vals_inv[3] = {255.f, 255.f, 255.f};
    ncnn::Net net;

    bool extract(const cv::Mat& src, cv::Mat& out);

    // std::vector<cv::Point2f> extractFeature(
    //     const cv::Mat& score,
    //     int ncellsize = 20,
    //     const std::vector<cv::Point2f>& vcurkps = std::vector<cv::Point2f>()
    // );
};