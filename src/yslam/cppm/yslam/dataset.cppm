export module dataset;

import cv;
import std;

export namespace Yslam {

class Dataset {
public:
    virtual void load(const std::string& path) = 0;
    int size() const { return timestamps.size(); }

public:
    std::vector<double> timestamps;
};

class Kitti : public Dataset {
public:
    std::vector<std::string> images_0;
    std::vector<std::string> images_1;

public:
    void load(const std::string& path) override
    {
        // Load KITTI dataset
        namespace fs = std::filesystem;
        std::println("Loading KITTI dataset from: {}", path);

        std::vector<std::tuple<double, std::string, std::string>> frame_paths;

        // Read timestamps
        fs::path timeFile = fs::path(path) / "times.txt";
        if (!fs::exists(timeFile)) {
            std::println("Error: times.txt not found at {}", timeFile.string());
            throw std::runtime_error("times.txt not found");
        }

        // Prepare image paths
        fs::path leftImageDir = fs::path(path) / "image_0";
        fs::path rightImageDir = fs::path(path) / "image_1";

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

                timestamps.push_back(t);
                images_0.push_back(leftImagePath.string());
                images_1.push_back(rightImagePath.string());

                i++;
            }
        }
    }
};

} // namespace Yslam
