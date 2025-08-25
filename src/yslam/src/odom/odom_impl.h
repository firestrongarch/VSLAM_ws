#pragma once
#include "extractor.h"
#include "frame.h"
#include "odom.h"

namespace Yslam {

class Odom::OdomImpl {
public:
    OdomImpl();
    ~OdomImpl();

    void run(std::string config_file);
    void stop();
    void readConfig(const std::string& config_file);
    void track(Frame frame);

private:
    std::string dataset_path_;
    std::unique_ptr<Extractor> extractor_ = std::make_unique<ORBExtractor>();
};

}