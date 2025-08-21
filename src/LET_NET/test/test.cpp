#include "letnet/letnet.h"
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <print>
#include <string>

class FeatureTracker {
public:
    FeatureTracker(const std::string& model_path, int cell_size)
        : letnet(model_path)
        , grid_size(cell_size)
    { // 添加网格大小参数
    }

    void readImage(const cv::Mat& _img)
    {
        cur_img = _img.clone();

        if (prev_img.empty()) {
            prev_img = cur_img.clone();
            extractFeatures();
            prev_pts = cur_pts; // 保存当前特征点作为前一帧特征点
            return;
        }

        if (!prev_pts.empty()) { // 只有当存在前一帧特征点时才进行跟踪
            trackFeatures();
            std::cout << "Tracked features: " << cur_pts.size() << std::endl;
        }

        // 当特征点数量太少时，提取新的特征点
        if (cur_pts.size() < 100) { // 可以调整这个阈值
            extractFeatures();
            std::cout << "Extracted new features, total: " << cur_pts.size() << std::endl;
        }

        // 更新图像和特征点
        prev_img = cur_img.clone();
        prev_pts = cur_pts;
    }

    void extractFeatures()
    {
        cv::Mat desc(480, 640, CV_8UC3), score(480, 640, CV_8UC1);
        std::cout << "extractFeatures cvtColor" << std::endl;
        letnet.extract(cur_img, desc, score);
        std::cout << "extractFeatures extract" << std::endl;
        // 使用命令行参数设置的网格大小
        std::vector<cv::Point2f> new_features = letnet.extractFeature(score, grid_size, cur_pts);
        std::cout << "extractFeatures letnet.extractFeature with grid size: " << grid_size << std::endl;
        cur_pts.insert(cur_pts.end(), new_features.begin(), new_features.end());
        std::cout << "extractFeatures cur_pts.insert" << std::endl;
    }

    void trackFeatures()
    {
        if (prev_pts.empty()) {
            return; // 如果没有特征点可跟踪，直接返回
        }

        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status;
        std::vector<float> err;

        cv::Mat prev_gray, cur_gray;
        cv::cvtColor(prev_img, prev_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cur_img, cur_gray, cv::COLOR_BGR2GRAY);

        try {
            cv::calcOpticalFlowPyrLK(prev_gray, cur_gray, prev_pts, next_pts, status, err);

            // 保存特征点轨迹
            cur_pts.clear();
            tracks.clear(); // 清除旧的轨迹
            for (size_t i = 0; i < status.size(); i++) {
                if (status[i]) {
                    cur_pts.push_back(next_pts[i]);
                    tracks.push_back(std::make_pair(prev_pts[i], next_pts[i]));
                }
            }
        } catch (const cv::Exception& e) {
            std::cout << "Tracking failed: " << e.what() << std::endl;
            cur_pts.clear(); // 清空当前特征点，下一步会重新提取
            tracks.clear();
        }
    }

    void drawTracking(cv::Mat& img)
    {
        // 绘制特征点轨迹线段
        for (const auto& track : tracks) {
            cv::line(img, track.first, track.second, cv::Scalar(0, 255, 0), 1);
        }

        // 绘制当前特征点
        for (const auto& pt : cur_pts) {
            // 绘制外圆
            // cv::circle(img, pt, 3, cv::Scalar(0, 0, 0), 1);
            // 绘制内点
            cv::circle(img, pt, 2, cv::Scalar(0, 255, 0), -1);
        }

        // 添加文本显示特征点数量
        // std::string text = "Features: " + std::to_string(cur_pts.size());
        // cv::putText(img, text, cv::Point(10, 20),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5,
        //             cv::Scalar(0, 255, 0), 1);
    }

private:
    LetNet letnet;
    int grid_size; // 添加网格大小成员变量
    cv::Mat prev_img;
    cv::Mat cur_img;
    std::vector<cv::Point2f> prev_pts;
    std::vector<cv::Point2f> cur_pts;
    std::vector<std::pair<cv::Point2f, cv::Point2f>> tracks; // 存储特征点轨迹
};

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " <model_path> <image_sequence_path> <grid_size>" << std::endl;
        std::cout << "Example: " << argv[0] << " model_dir image_sequence 20" << std::endl;
        return -1;
    }

    int grid_size = std::stoi(argv[3]); // 从命令行获取网格大小
    if (grid_size < 5 || grid_size > 100) {
        std::cout << "Grid size should be between 5 and 100" << std::endl;
        return -1;
    }

    // 初始化特征跟踪器，传入网格大小参数
    FeatureTracker tracker(argv[1], grid_size);
    std::println("FeatureTracker initialized.");

    // 打开视频或图像序列
    cv::VideoCapture cap(argv[2] + std::string("/%06d.png"));
    if (!cap.isOpened()) {
        std::cout << "Failed to open video file!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    int frame_count = 0;
    while (cap.read(frame)) {
        std::cout << "\nProcessing frame " << frame_count++ << std::endl;

        // 特征点跟踪
        tracker.readImage(frame);

        // 绘制跟踪结果
        tracker.drawTracking(frame);

        // 显示结果
        cv::imshow("Tracking", frame);

        char key = cv::waitKey(1); // 改为1ms延时，使显示更流畅
        if (key == 27)
            break; // ESC键退出
    }
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
