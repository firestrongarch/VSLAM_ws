#include "dataset_reader/factories.hpp"
#include "dataset_reader/kitti_dataset.hpp"  // 引入 KittiDataset 头文件
#include <iostream>
#include <Poco/Environment.h>
#include <Poco/Path.h>
#include <thread>

int main() {
    // 注册 KittiDataset 类型
    fsa::DatasetFactory::register_type<fsa::KittiDataset>("kitti");

    auto dataset = fsa::DatasetFactory::create("kitti", "~/data/kitti/00");
    while (dataset->has_next()) {
        auto frame = dataset->load_next();
        std::cout<< frame->left_image_path <<"\n";
        using namespace std::chrono;
        std::this_thread::sleep_for(30ms);
    }
}