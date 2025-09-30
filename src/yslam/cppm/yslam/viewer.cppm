export module viewer;

import cv;
import types;
import implot3d;
import std;

export namespace Yslam {

class Viewer {
public:
    void Run();
    // void Imgui();
    static Viewer& GetInstance();

    float track_fps_ = 0;

private:
    Viewer() = default;
    ~Viewer() = default;
    Viewer(const Viewer&) = delete;
    Viewer(Viewer&&) = delete;
};

Viewer& Viewer::GetInstance()
{
    static Viewer instance;
    return instance;
}

void Viewer::Run()
{
    std::shared_ptr<Frame> frame;
    Map::GetInstance().frame_queue_.pop(frame);
    static cv::Mat image_kps = frame->last->img0.clone();
    if (frame) {
        image_kps = frame->img0.clone();
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
    }

    ImGui::SetNextWindowPos(ImGui::ImVec2(0, 0), ImGui::ImGuiCond_::ImGuiCond_FirstUseEver);
    // ImGui::SetNextWindowSize(ImGui::ImVec2(640, 480), ImGui::ImGuiCond_::ImGuiCond_FirstUseEver);
    // 显示一个ImGui窗口
    ImGui::Begin("Camera Feed");
    ImGui::Text("Avg Odom FPS: %.2f", track_fps_);
    // 显示OpenCV图像
    ImPlot3D::ShowMat(image_kps);
    ImGui::End();

    auto poses_vo = Map::GetInstance().GetAllPosesVO();
    int size1 = poses_vo.size();
    static double xs1[5000], ys1[5000], zs1[5000];
    for (int i = 0; i < size1; ++i) {
        const auto& pose = poses_vo[i];
        xs1[i] = pose.at<double>(0, 3);
        ys1[i] = pose.at<double>(1, 3);
        zs1[i] = pose.at<double>(2, 3);
    }

    auto kfs = Map::GetInstance().all_key_frames_;
    int size2 = kfs.size();
    static double xs2[5000], ys2[5000], zs2[5000];
    for (int i = 0; i < size2; ++i) {
        const auto& pose = kfs.at(i)->T_wc;
        xs2[i] = pose.at<double>(0, 3);
        ys2[i] = pose.at<double>(1, 3);
        zs2[i] = pose.at<double>(2, 3);
    }

    // auto mps = Map::GetInstance().all_map_points_;
    // int size3 = mps.size();
    // static std::vector<double> xs3, ys3, zs3;
    // for (auto [id, mp] : mps) {
    //     xs3.push_back(mp->x);
    //     ys3.push_back(mp->y);
    //     zs3.push_back(mp->z);
    // }

    ImGui::Begin("ImPlot3D Demo");
    ImGui::Spacing();
    if (ImPlot3D::BeginPlot("Line Plots")) {
        ImPlot3D::SetupBoxInitialRotation(360, 0);
        // ImPlot3D::SetupBoxRotation(0, 180);
        ImPlot3D::SetupBoxScale(1.5, 1.5, 1.5);
        ImPlot3D::SetupAxes("x", "y", "z",
            ImPlot3D::ImPlot3DAxisFlags_::ImPlot3DAxisFlags_AutoFit,
            ImPlot3D::ImPlot3DAxisFlags_::ImPlot3DAxisFlags_AutoFit,
            ImPlot3D::ImPlot3DAxisFlags_::ImPlot3DAxisFlags_AutoFit);
        // ImPlot3D::SetNextMarkerStyle(ImPlot3D::ImPlot3DMarker_::ImPlot3DMarker_Circle);
        ImPlot3D::PlotLine("VO", xs1, ys1, zs1, size1);
        ImPlot3D::PlotLine("KF", xs2, ys2, zs2, size2);
        // ImPlot3D::PlotScatter("Mps", xs3.data(), ys3.data(), zs3.data(), xs3.size());
        // ImPlot3D::PlotLine("g(x)", xs2, ys2, zs2, 20, ImPlot3D::ImPlot3DLineFlags_::ImPlot3DLineFlags_Segments);
        ImPlot3D::EndPlot();
    }
    ImGui::End();
}

}