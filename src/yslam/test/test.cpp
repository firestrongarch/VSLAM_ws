#include "yslam/letnet_extractor.h"
#include "yslam/viewer.h"

int main()
{
    Yslam::LetNetExtractor extractor;
    extractor.load("E:/CPP/VSLAM_ws/src/config_pkg/model");

    cv::Mat img = cv::imread("E:/datasets/Kitti00-10/09/image_0/000136.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        return -1;
    }

    Yslam::Frame frame;
    frame.img0 = img;

    extractor.extract({ .img = frame.img0,
        .mask = frame.mask,
        .pts = frame.pts0,
        .kps = frame.kps0,
        .desc = frame.desc0 });

    std::cout << "Extracted " << frame.pts0.size() << " keypoints." << std::endl;

    Yslam::LetNetViewer viewer;
    viewer.view({ frame.img0, frame.pts0 });

    return 0;
}