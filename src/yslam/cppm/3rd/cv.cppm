module;
#include <opencv2/core/hal/hal.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/opencv.hpp>

export module cv;

export namespace cv {

using cv::Mat;
using cv::Mat_;
using cv::MatExpr;
using cv::Vec4d;

auto MatMul(const Mat& a, const Mat& b) -> Mat
{
    return a * b;
}

using cv::circle;
using cv::cvtColor;
using cv::rectangle;

using cv::imread;
using cv::imshow;
using cv::waitKey;

using cv::BFMatcher;
using cv::DMatch;
using cv::KeyPoint;
using cv::Point2d;
using cv::Point2f;
using cv::Point3d;

using cv::drawKeypoints;
using cv::drawMatches;
using cv::FastFeatureDetector;
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

using cv::findFundamentalMat;
using cv::Quatd;
using cv::Rodrigues;
using cv::solvePnPRansac;
using cv::triangulatePoints;

// using CV_8UC1;
using cv::Ptr;

using cv::COLOR_BGR2GRAY;
using cv::COLOR_GRAY2BGR;
using cv::FILLED;

const int MAT_8UC1 = CV_8UC1;
const int MAT_8UC3 = CV_8UC3;
const int MAT_32F = CV_32F;
const int MAT_64F = CV_64F;
const int MAT_8U = CV_8U;
using cv::FM_RANSAC;
using cv::NORM_HAMMING;
using uchar = unsigned char;

// core/utility.hpp
void parallel_for(const Range& range, std::function<void(const Range&)> functor, double nstripes = -1.)
{
    cv::parallel_for_(range, ParallelLoopBodyLambdaWrapper(functor), nstripes);
}

using cv::ParallelLoopBody;

cv::Mat p2d_mat = (cv::Mat_<double>(3, 1) << 1, 1, 1);

namespace hal {
    using cv::hal::normHamming;
}

};