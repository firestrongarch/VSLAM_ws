#pragma once
#include "yslam/letnet_extractor.h"
#include <opencv2/opencv.hpp>

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
        }

        std::vector<uchar> status;
        std::vector<float> err;

        params.pts1.clear();
        cv::calcOpticalFlowPyrLK(
            params.img0,
            params.img1,
            params.pts0,
            params.pts1,
            status,
            err,
            cv::Size(21, 21),
            7);
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
        // Implement LK tracking here
    }
};

class LetNetTracker : public Tracker {
public:
    void track(TrackerParams params) override
    {
        static LetNetExtractor extractor;

        cv::Mat desc0, desc1;
        extractor.extractDescriptor(params.img0, desc0);
        extractor.extractDescriptor(params.img1, desc1);

        static LkTracker lk;
        lk.track({ .img0 = desc0,
            .img1 = desc1,
            .pts0 = params.pts0,
            .pts1 = params.pts1 });
    }
};

}