#include "yslam/frame.h"
#include "yslam/letnet_extractor.h"
#include "yslam/viewer.h"

/**
 * @brief 主函数，用于加载图像，提取特征点，并显示提取结果
 *
 * 该函数首先加载一个特征提取器，然后加载一张灰度图像。如果图像加载失败，则输出错误信息并返回-1。
 * 接着，创建一个帧对象，将加载的图像赋值给帧对象的图像成员变量。然后，使用特征提取器提取图像中的特征点，
 * 包括关键点、描述子和掩码。最后，输出提取的关键点数量，并使用视图器显示图像和提取的关键点。
 *
 * @return 0 表示程序正常结束，-1 表示图像加载失败
 */
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