module;
#include <opencv2/opencv.hpp>

export module cv;

export namespace cv {

using cv::Mat;

using cv::circle;
using cv::cvtColor;

using cv::imread;
using cv::imshow;
using cv::waitKey;

using cv::KeyPoint;
using cv::Point2f;

using cv::ORB;
using cv::SIFT;
};