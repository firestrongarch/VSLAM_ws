#pragma once

#include <opencv2/opencv.hpp>

namespace Yslam {

class Extractor {
public:
    struct Param {
        const cv::Mat& img;
        const cv::Mat& mask;
        std::vector<cv::Point2f>& pts;
    };
    virtual void extract(Param params) = 0;
    virtual ~Extractor() = default;
};

class ORBExtractor : public Extractor {
public:
    void extract(Param params) override
    {
        // Implement ORB feature extraction here
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        orb->detectAndCompute(params.img, params.mask, kps, desc);
    }
};

} // namespace Yslam