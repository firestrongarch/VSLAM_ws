#pragma once
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <print>
#include <sstream>
#include <vector>

namespace Yslam {

inline auto loadKittiDataset(const std::string& dataset_path)
{
    namespace fs = std::filesystem;
    std::println("Loading KITTI dataset from: {}", dataset_path);

    std::vector<std::tuple<double, std::string, std::string>> frame_paths;

    // Read timestamps
    fs::path timeFile = fs::path(dataset_path) / "times.txt";
    if (!fs::exists(timeFile)) {
        std::println("Error: times.txt not found at {}", timeFile.string());
        throw std::runtime_error("times.txt not found");
    }

    // Prepare image paths
    fs::path leftImageDir = fs::path(dataset_path) / "image_0";
    fs::path rightImageDir = fs::path(dataset_path) / "image_1";

    std::ifstream fTimes(timeFile);
    while (fTimes) {
        static int i = 0;
        std::string s;
        std::getline(fTimes, s);
        if (!s.empty()) {
            std::stringstream ss(s);
            double t;
            ss >> t;

            std::stringstream ss_path;
            ss_path << std::setfill('0') << std::setw(6) << i;
            fs::path leftImagePath = leftImageDir / (ss_path.str() + ".png");
            fs::path rightImagePath = rightImageDir / (ss_path.str() + ".png");
            frame_paths.push_back({ t, leftImagePath.string(), rightImagePath.string() });

            i++;
        }
    }

    return frame_paths;
}

}