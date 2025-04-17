#include "dataset_reader/kitti_dataset.hpp"
#include <iomanip>
#include <Poco/FileStream.h>  // 引入Poco库的文件输入流
#include <Poco/StreamCopier.h>     // 引入Poco库的流复制工具
#include <Poco/File.h>             // 引入Poco库的文件操作
#include <Poco/StringTokenizer.h>  // 引入Poco库的字符串分割工具
#include <Poco/Path.h>             // 引入Poco库的路径操作

namespace fsa {

KittiDataset::KittiDataset(const std::string& path): dataset_path_(Poco::Path::expand(path)) {
    std::string strPathTimeFile = dataset_path_ + "/times.txt";

    // 使用Poco::FileInputStream代替std::ifstream
    Poco::FileInputStream fTimes(strPathTimeFile);
    if (!fTimes.good()) {
        throw std::runtime_error("Failed to open times.txt");
    }

    // 使用Poco::StreamCopier::copyToString来读取文件内容
    std::string fileContent;
    Poco::StreamCopier::copyToString(fTimes, fileContent);

    // 使用Poco::StringTokenizer来分割文件内容
    Poco::StringTokenizer tokenizer(fileContent, "\n", Poco::StringTokenizer::TOK_IGNORE_EMPTY | Poco::StringTokenizer::TOK_TRIM);
    for (const auto& token : tokenizer) {
        try {
            double t = std::stod(token);
            timestamps_.push_back(t);
        } catch (const std::invalid_argument& e) {
            // 忽略无效的行
        }
    }

    std::string strPrefixLeft = dataset_path_ + "/image_0/";
    std::string strPrefixRight = dataset_path_ + "/image_1/";

    const size_t nTimes = timestamps_.size();
    path_left_.resize(nTimes);
    path_right_.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        path_left_[i] = strPrefixLeft + ss.str() + ".png";
        path_right_[i] = strPrefixRight + ss.str() + ".png";
    }
    fTimes.close();

    // 初始化 total_frames_
    total_frames_ = timestamps_.size();
}

std::optional<Frame> KittiDataset::load_next() {
    if (current_index_ >= total_frames_) {
        return std::nullopt;
    }

    Frame frame;
    frame.timestamp = timestamps_[current_index_];
    frame.left_image_path = path_left_[current_index_];
    frame.right_image_path = path_right_[current_index_];

    current_index_++;
    return frame;
}

bool KittiDataset::has_next() const{ 
    return current_index_ < total_frames_; 
}

}

