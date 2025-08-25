#pragma once

#include "frame.h"

namespace Yslam {

class Extractor {
public:
    virtual void extract(Frame& frame) = 0;
    virtual ~Extractor() = default;
};

class ORBExtractor : public Extractor {
public:
    void extract(Frame& frame) override
    {
        // Implement ORB feature extraction here
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(frame.img0, cv::Mat(), frame.kps0, frame.desc0);
    }
};

} // namespace Yslam