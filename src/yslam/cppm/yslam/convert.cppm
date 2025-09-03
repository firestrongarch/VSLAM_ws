export module convert;

import std;
import cv;

export namespace Yslam {

auto kpsToPts(const std::vector<cv::KeyPoint>& keypoints)
{
    std::vector<cv::Point2f> points;
    points.reserve(keypoints.size());
    for (const auto& kp : keypoints) {
        points.emplace_back(kp.pt);
    }
    return points;
}

auto ptsToKps(const std::vector<cv::Point2f>& points)
{
    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(points.size());
    for (const auto& pt : points) {
        keypoints.emplace_back(pt, 1.f); // Assuming a default size of 1.0
    }
    return keypoints;

} // namespace Yslam
}
