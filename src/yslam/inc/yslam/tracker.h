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
        if (params.pts0.empty()) {
            throw std::runtime_error("Error: Keypoints are empty.");
        } else {
            std::vector<uchar> status;
            std::vector<float> err;

            params.pts1.clear();
            cv::calcOpticalFlowPyrLK(params.img0, params.img1, params.pts0, params.pts1, status, err);
            // Filter points based on status
            std::vector<cv::Point2f> filtered_pts0;
            std::vector<cv::Point2f> filtered_pts1;
            for (size_t i = 0; i < status.size(); i++) {
                if (status[i]) {
                    filtered_pts0.push_back(params.pts0.at(i));
                    filtered_pts1.push_back(params.pts1.at(i));
                }
            }

            params.pts0 = filtered_pts0;
            params.pts1 = filtered_pts1;
        }
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