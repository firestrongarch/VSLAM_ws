#include "dataset_reader/dataset.hpp"

namespace fsa {

// kitti_dataset.hpp
class KittiDataset : public Dataset {
public:
    explicit KittiDataset(const std::string& path);
    std::optional<Frame> load_next() override;
    bool has_next() const override;

private:
    std::string dataset_path_;
    uint64_t current_index_ = 0;    // <- 索引成员在此声明
    uint64_t total_frames_ = 0;
    // 其他数据集特定成员（如文件列表）
    std::vector<std::string> image_files_; 
    std::vector<double> timestamps_;
    std::vector<std::string> path_left_;
    std::vector<std::string> path_right_;
};

}
