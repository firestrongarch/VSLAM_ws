#include "dataset_reader/kitti_dataset.hpp"

KittiDataset::KittiDataset(const std::string& path) {
    // 初始化KITTI数据集解析逻辑
}

std::optional<Frame> KittiDataset::load_next() {
    Frame frame;
    // 解析下一帧数据
    if (1) return frame;
    return std::nullopt;
}

bool KittiDataset::has_next() const{ 
    return current_index_ < total_frames_; 
}
