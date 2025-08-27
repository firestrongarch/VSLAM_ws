#pragma once
#include "yslam/frame.h"
namespace Yslam {

struct TrackerParams {
    const cv::Mat& img0;
    const cv::Mat& img1;
    std::vector<cv::Point2f>& pts0;
    std::vector<cv::Point2f>& pts1;
};

class Tracker {
public:
    virtual void track(TrackerParams params) = 0;
    virtual ~Tracker() = default;
};

class LkTracker : public Tracker {
public:
    void track(TrackerParams params) override
    {

        // Implement LK tracking here
    }
};

class LetNetTracker : public Tracker {
public:
    void track(TrackerParams params) override
    {
        // Implement LetNet specific tracking here
    }
};

}