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
import backend;
// import tracker;

export namespace Yslam {

class Odom {
public:
    // Public interface methods
    void run(const std::string& config_file);
    void stop();
    void readConfig(const std::string& config_file);

    bool NeedNewKeyFrame();

private:
    std::shared_ptr<Kitti> kitti_;
    std::shared_ptr<Frame> current_frame { nullptr };
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
    std::jthread backend_thread([]() {
        Backend::GetInstance().Run();
    });
    std::printf("Starting odometry...\n");
    // std::this_thread::sleep_for(std::chrono::seconds(1)); // Ensure the viewer thread starts first

    auto extractor = cv::ORB::create();
    auto cam = std::make_shared<Stereo>();
    for (int i = 0; i < kitti_->size(); ++i) {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        cv::Mat left_image = cv::imread(kitti_->images_0[i], 0);
        cv::Mat right_image = cv::imread(kitti_->images_1[i], 0);
        current_frame = std::make_shared<Frame>(
            Frame {
                .ID = i,
                .timestamp = kitti_->timestamps[i],
                .img0 = left_image,
                .img1 = right_image });

        if (current_frame->last == nullptr) {
            cam->Init(current_frame);
            Map::GetInstance().InsertKeyFrame(current_frame);
            continue;
        }

        cam->Track({ .img0 = current_frame->last->img0,
            .img1 = current_frame->img0,
            .kps0 = current_frame->last->kps,
            .kps1 = current_frame->kps });
        for (auto& kp : current_frame->kps) {
            auto mp = kp.map_point.lock();
            if (mp) {
                mp->obs.push_back(current_frame);
            }
        }

        static Optimizer opt;
        opt.PerspectiveNPoint(current_frame);

        // If tracked points are less than a threshold, detect new keypoints
        if (NeedNewKeyFrame()) {
            cam->Extract3d(current_frame);
            Map::GetInstance().InsertKeyFrame(current_frame);
        }

        current_frame->last = current_frame;
        Map::GetInstance().frame_queue_.push(current_frame);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        static float total_time = 0.0;
        total_time += elapsed;
        float avg_time = total_time / (i + 1);
        Viewer::GetInstance().track_fps_ = 1000.0 / avg_time;
    }
}

bool Odom::NeedNewKeyFrame()
{
    float track_last_ratio = static_cast<float>(current_frame->kps.size()) / static_cast<float>(current_frame->last->kps.size());
    if (track_last_ratio < 0.7) {
        return true;
    }

    float track_kf_ratio = static_cast<float>(current_frame->kps.size()) / static_cast<float>(Map::GetInstance().ref_kf_->kps.size());
    if (track_kf_ratio < 0.25) {
        return true;
    }

    // Implementation for determining whether a new keyframe is needed
    return false;
}

void Odom::stop()
{
    // std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

} // namespace Yslam
