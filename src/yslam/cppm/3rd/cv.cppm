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
using cv::Point2d;
using cv::Point2f;
using cv::Point3d;

using cv::drawKeypoints;
using cv::drawMatches;
using cv::Feature2D;
using cv::ORB;
using cv::SIFT;

using cv::InputArray;
using cv::OutputArray;

using cv::calcOpticalFlowPyrLK;
using cv::minMaxLoc;
using cv::Point;
using cv::Range;
using cv::Rect;
using cv::Scalar;
using cv::Size;

using cv::triangulatePoints;

// using CV_8UC1;
using cv::Ptr;

const int MAT_8UC1 = CV_8UC1;
const int MAT_8UC3 = CV_8UC3;
};