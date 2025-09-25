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
    boost::lockfree::spsc_queue<std::shared_ptr<Frame>> frame_queue_ { 5000 };
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
    std::jthread view_thread2(ImPlot3D::Run, [this]() { this->Imgui(); });

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
        frame_queue_.push(current_frame);
    }
}

void Odom::Imgui()
{
    std::shared_ptr<Frame> frame;
    frame_queue_.pop(frame);
    if (!frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
    }
    auto image_kps = frame->img0.clone();
    auto kps = frame->kps;
    std::println("Frame ID: {}, Keypoints: {}", frame->ID, kps.size());
    cv::cvtColor(image_kps, image_kps, cv::COLOR_GRAY2BGR);
    // cv::drawKeypoints(image_kps, ptsToKps(frame->last->pts), image_kps, cv::Scalar(0, 255, 0));
    for (const auto& kp : kps) {
        cv::Point2f pt1, pt2;
        pt1.x = kp.pt.x - 5;
        pt1.y = kp.pt.y - 5;
        pt2.x = kp.pt.x + 5;
        pt2.y = kp.pt.y + 5;
        cv::rectangle(image_kps, pt1, pt2, cv::Scalar(0, 255, 0));
        cv::circle(image_kps, kp.pt, 2, cv::Scalar(0, 255, 0), cv::FILLED);
    }

    // 显示一个ImGui窗口
    ImGui::Begin("Camera Feed");
    // 显示OpenCV图像
    ImPlot3D::ShowMat(image_kps);
    ImGui::End();

    ImGui::Begin("ImPlot3D Demo");
    ImGui::Spacing();

    auto poses = Map::GetInstance().GetAllPosesVO();

    int size = poses.size();
    static double xs1[5000], ys1[5000], zs1[5000];
    for (int i = 0; i < size; ++i) {
        const auto& pose = poses[i];
        xs1[i] = pose.at<double>(0, 3);
        ys1[i] = pose.at<double>(1, 3);
        zs1[i] = pose.at<double>(2, 3);
    }
    if (ImPlot3D::BeginPlot("Line Plots")) {
        ImPlot3D::SetupBoxInitialRotation(90, 0);
        // ImPlot3D::SetupBoxRotation(0, 180);
        ImPlot3D::SetupBoxScale(1.5, 1.5, 1.5);
        ImPlot3D::SetupAxes("x", "y", "z");
        // ImPlot3D::SetNextMarkerStyle(ImPlot3D::ImPlot3DMarker_::ImPlot3DMarker_Circle);
        ImPlot3D::PlotLine("f(x)", xs1, ys1, zs1, 5000);
        
        // ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3D::ImPlot3DLineFlags_::ImPlot3DLineFlags_Segments);
        ImPlot3D::EndPlot();
    }
    ImGui::End();
}

void Odom::view()
{
    while (1) {
        std::shared_ptr<Frame> frame;
        frame_queue_.pop(frame);
        if (!frame) {
            cv::waitKey(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        auto image_kps = frame->img0.clone();
        auto kps = frame->kps;
        std::println("Frame ID: {}, Keypoints: {}", frame->ID, kps.size());
        cv::cvtColor(image_kps, image_kps, cv::COLOR_GRAY2BGR);
        // cv::drawKeypoints(image_kps, ptsToKps(frame->last->pts), image_kps, cv::Scalar(0, 255, 0));
        for (const auto& kp : kps) {
            cv::Point2f pt1, pt2;
            pt1.x = kp.pt.x - 5;
            pt1.y = kp.pt.y - 5;
            pt2.x = kp.pt.x + 5;
            pt2.y = kp.pt.y + 5;
            cv::rectangle(image_kps, pt1, pt2, cv::Scalar(0, 255, 0));
            cv::circle(image_kps, kp.pt, 2, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        cv::imshow("Left Image", image_kps);
        cv::waitKey(1);
    }

    // Implementation for viewing the odometry
}

void Odom::stop()
{
    // std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

} // namespace Yslam
