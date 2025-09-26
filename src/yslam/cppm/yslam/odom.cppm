export module odom;

import cv;
import dataset;
import types;
import std;
import toml;
import convert;
import lockfree;
import camera;
import optimizer;
import implot3d;
import viewer;
// import tracker;

export namespace Yslam {

class Odom {
public:
    // Public interface methods
    void run(const std::string& config_file);
    void stop();
    void readConfig(const std::string& config_file);
    void view();
    void Imgui();

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

    // std::jthread view_thread(&Odom::view, this);
    std::jthread view_thread2(ImPlot3D::Run, [this]() { Viewer::GetInstance().Run(); });
    std::printf("Starting odometry...\n");
    // std::this_thread::sleep_for(std::chrono::seconds(1)); // Ensure the viewer thread starts first

    auto extractor = cv::ORB::create();
    auto cam = std::make_shared<Stereo>();
    for (int i = 0; i < kitti_->size(); ++i) {
        cv::Mat left_image = cv::imread(kitti_->images_0[i], 0);
        cv::Mat right_image = cv::imread(kitti_->images_1[i], 0);
        auto current_frame = std::make_shared<Frame>(
            Frame {
                .ID = i,
                .timestamp = kitti_->timestamps[i],
                .img0 = left_image,
                .img1 = right_image });

        if (current_frame->last == nullptr) {
            cam->Init(current_frame);
            continue;
        }

        cam->Track({ .img0 = current_frame->last->img0,
            .img1 = current_frame->img0,
            .kps0 = current_frame->last->kps,
            .kps1 = current_frame->kps });

        static Optimizer opt;
        opt.PerspectiveNPoint(current_frame);

        // If tracked points are less than a threshold, detect new keypoints
        if (current_frame->kps.size() < 100) {
            // cv::Mat mask;
            // for (const auto& kp : current_frame->kps) {
            //     cv::circle(mask, kp.pt, 10, cv::Scalar(0), -1);
            // }
            // std::vector<cv::KeyPoint> kps;
            // extractor->detect(current_frame->img0, kps, mask);
            // for (const auto& kp : kps) {
            //     current_frame->kps.push_back(KeyPoint(kp));
            // }
            cam->Extract3d(current_frame);
        }

        current_frame->last = current_frame;
        Map::GetInstance().frame_queue_.push(current_frame);
    }
}

void Odom::stop()
{
    // std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

} // namespace Yslam
