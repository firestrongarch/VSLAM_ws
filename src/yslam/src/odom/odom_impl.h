#pragma once
#include "yslam/frame.h"
#include "yslam/letnet_extractor.h"
#include "yslam/odom.h"
#include "yslam/viewer.h"

namespace Yslam {

class Odom::OdomImpl {
public:
    OdomImpl();
    ~OdomImpl();

    void run(std::string config_file);
    void stop();
    void readConfig(const std::string& config_file);
    void track(std::shared_ptr<Frame> frame);

private:
    std::string dataset_path_;
    std::unique_ptr<LetNetExtractor> extractor_ = std::make_unique<LetNetExtractor>();
    std::unique_ptr<Viewer> viewer_ = std::make_unique<LetNetViewer>();
};

}