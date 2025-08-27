#pragma once

#include "yslam/frame.h"
#include <opencv2/opencv.hpp>

namespace Yslam {

struct ViewerParams {
    const cv::Mat& img;
    const std::vector<cv::Point2f>& pts;
};

class Viewer {
public:
    virtual void view(ViewerParams params) = 0;
    virtual ~Viewer() = default;
};

class LetNetViewer : public Viewer {
public:
    void view(ViewerParams params) override
    {
        auto img = params.img.clone();
        if (img.empty()) {
            throw std::runtime_error("Error: Input image is empty.");
        }
        if (params.pts.empty()) {
            throw std::runtime_error("Error: No keypoints extracted.");
        }
        if (img.channels() == 1) {
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        }

        for (const auto& pt : params.pts) {
            cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
        }

        // std::cout << "desc size : " << params.desc.size() << "\n";
        std::cout << "img0 size : " << params.img.size() << "\n";
        // Implement LetNet specific viewing here
        cv::imshow("LetNet Keypoints", img);
        cv::waitKey(0);
    }
};

} // namespace Yslam