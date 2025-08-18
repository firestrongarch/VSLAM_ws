#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat image = cv::imread("E:/datasets/Kitti00-10/00/image_0/000000.png");
    if (image.empty()) {
        std::cerr << "Failed to load image" << std::endl;
        return -1;
    }

    cv::imshow("Display Image", image);
    cv::waitKey(0);
    return 0;
}