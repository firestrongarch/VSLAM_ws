#pragma once

#include "yslam/frame.h"
#include <opencv2/opencv.hpp>

namespace Yslam {

class Viewer {
public:
    virtual void view(const Frame& frame) = 0;
    virtual ~Viewer() = default;
};

class LetNetViewer : public Viewer {
public:
    void view(const Frame& frame) override
    {
        auto img = frame.img0.clone();
        if (img.empty()) {
            throw std::runtime_error("Error: Input image is empty.");
        }
        if (frame.pts0.empty()) {
            throw std::runtime_error("Error: No keypoints extracted.");
        }
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }

        for (const auto& pt : frame.pts0) {
            cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
        }

        std::cout << "desc size : " << frame.desc0.size() << "\n";
        std::cout << "img0 size : " << frame.img0.size() << "\n";
        // Implement LetNet specific viewing here
        cv::imshow("LetNet Keypoints", img);
        cv::waitKey(0);
    }
};

} // namespace Yslam