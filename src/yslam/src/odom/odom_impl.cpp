#include "odom_impl.h"
#include "toml++/toml.h"
// #include "toml.hpp"
#include "kitti.h"
#include <print>

namespace Yslam {

Odom::OdomImpl::OdomImpl() = default;

Odom::OdomImpl::~OdomImpl() = default;

void Odom::OdomImpl::run(std::string config_file)
{
    readConfig(config_file);
    // Implementation for running the odometry with the given config file
}

void Odom::OdomImpl::readConfig(const std::string& config_file)
{
    auto config = toml::parse_file(config_file);

    std::println("datasets path: {}", config["dataset"]["path"].value_or("unknown"));
    // auto config = toml::parse(config_file);
    // std::cout << config << "\n";

    std::string dataset_path = config["dataset"]["path"].value_or("unknown");

    auto frame_paths = loadKittiDataset(dataset_path);
}

void Odom::OdomImpl::stop()
{
    std::println("Stopping odometry...");
    // Implementation for stopping the odometry
}

}