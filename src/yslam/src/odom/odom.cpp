#include "yslam/odom.h"
#include "odom_impl.h"
#include <string>

namespace Yslam {

Odom::Odom()
    : impl_(std::make_unique<OdomImpl>())
{
}

Odom::~Odom() = default;

void Odom::run(std::string config_file)
{
    impl_->run(std::move(config_file));
}

void Odom::stop()
{
    impl_->stop();
}

}