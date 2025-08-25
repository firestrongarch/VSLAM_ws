#pragma once

#include <memory>
namespace Yslam {

class Odom {
private:
    struct OdomImpl;
    std::unique_ptr<OdomImpl> impl_;

public:
    Odom();
    ~Odom();

    // Disable copy and move semantics
    Odom(const Odom&) = delete;
    Odom& operator=(const Odom&) = delete;
    Odom(Odom&&) = delete;
    Odom& operator=(Odom&&) = delete;

    // Public interface methods
    void run(std::string config_file);
    void stop();
};

}