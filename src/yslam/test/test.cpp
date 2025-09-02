import letnet;

int main()
{
    auto letnet = Yslam::LetNet::create("E:/CPP/VSLAM_ws/src/config_pkg/model");
    cv::Mat image = cv::imread("E:/datasets/Kitti00-10/09/image_0/000155.png", 0);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    letnet->detectAndCompute(image, cv::Mat(), keypoints, descriptors);

    auto image_kps = image.clone();
    cv::drawKeypoints(image_kps, keypoints, image_kps, cv::Scalar(0, 255, 0));

    cv::imshow("Image", descriptors);
    cv::imshow("Image_kps", image_kps);
    cv::waitKey(0);
    return 0;
}