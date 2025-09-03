export module odom;

import cv;
import dataset;
import frame;
import std;
import toml;
import convert;
// import tracker;

export namespace Yslam {

class Odom {
public:
    // Public interface methods
    void run(const std::string& config_file);
    void stop();
    void readConfig(const std::string& config_file);
    void track(std::shared_ptr<Frame> frame);

    void init(std::shared_ptr<Frame> frame);
    void Lk(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point2f>& pts0, std::vector<cv::Point2f>& pts1);

private:
    std::shared_ptr<Kitti> kitti_;
    // std::shared_ptr<Tracker> tracker_;
};

void Odom::readConfig(const std::string& config_file)
{
    auto config = toml::parse_file(config_file);

    std::string dataset_path = config["dataset"]["path"].value_or("unknown");
    kitti_ = std::make_shared<Kitti>();
    kitti_->load(dataset_path);
}

void Odom::run(const std::string& config_file)
{
    readConfig(config_file);
    // Implementation for running the odometry with the given config file

    auto orb = cv::ORB::create();
    for (int i = 0; i < kitti_->size(); ++i) {
        auto current_frame = std::make_shared<Frame>();

        cv::Mat left_image = cv::imread(kitti_->images_0[i], 0);
        cv::Mat right_image = cv::imread(kitti_->images_1[i], 0);
        current_frame->img0 = left_image;
        current_frame->img1 = right_image;

        if (current_frame->pts.empty()) {
            std::vector<cv::KeyPoint> kps;
            orb->detect(current_frame->img0, kps);
            current_frame->pts = kpsToPts(kps);
        }

        if (current_frame->last) {
            Lk(current_frame->last->img0, current_frame->img0, current_frame->last->pts, current_frame->pts);
        }

        if (current_frame->pts.size() < 100) {
            cv::Mat mask;
            for (auto& pt : current_frame->pts) {
                cv::circle(mask, pt, 10, cv::Scalar(0), -1);
            }
            std::vector<cv::KeyPoint> kps;
            orb->detect(current_frame->img0, kps, mask);
            auto pts = kpsToPts(kps);
            current_frame->pts.insert(current_frame->pts.end(), pts.begin(), pts.end());
        }

        current_frame->last = current_frame;
        auto image_kps = current_frame->img0.clone();
        cv::drawKeypoints(image_kps, ptsToKps(current_frame->pts), image_kps, cv::Scalar(0, 255, 0));
        cv::imshow("Left Image", image_kps);
        cv::waitKey(1);
    }
}

void Odom::Lk(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point2f>& pts0, std::vector<cv::Point2f>& pts1)
{
    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        img0,
        img1,
        pts0,
        pts1,
        status,
        err,
        cv::Size(21, 21),
        7);

    // Filter points based on status
    std::vector<cv::Point2f> filtered_pts0;
    std::vector<cv::Point2f> filtered_pts1;
    for (int i = 0; i < status.size(); i++) {
        if (status[i]) {
            filtered_pts0.push_back(pts0.at(i));
            filtered_pts1.push_back(pts1.at(i));
        }
    }

    pts0 = filtered_pts0;
    pts1 = filtered_pts1;
}

void Odom::stop()
{
    // std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

} // namespace Yslam
