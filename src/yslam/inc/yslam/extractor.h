#pragma once

#include "yslam/frame.h"

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
        orb->detectAndCompute(frame.img0, frame.mask, frame.kps0, frame.desc0);
    }
};

} // namespace Yslam