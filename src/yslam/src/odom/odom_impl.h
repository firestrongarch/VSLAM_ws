#pragma once
#include "odom.h"

namespace Yslam {

class Odom::OdomImpl {
public:
    OdomImpl();
    ~OdomImpl();

    void run(std::string config_file);
    void stop();
    void readConfig(const std::string& config_file);
};

}