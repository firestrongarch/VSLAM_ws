#pragma once
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

namespace vslam {
class Tracker{
public:
    Tracker() = default;
    void RunBinocular(const cv::Mat &left_image, const cv::Mat &right_iamge,
                      const double timestamp);
    // void SetMap(const Map::Ptr map);
    // void SetUiPangolin(const UiPangolin::Ptr ui_pangolin);
};

}
