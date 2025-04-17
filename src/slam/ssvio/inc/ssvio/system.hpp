#ifndef SSVIO_SYSTEM_HPP
#define SSVIO_SYSTEM_HPP

#include "ssvio/setting.hpp"
#include "ui/pangolin_window.hpp"
#include "glog/logging.h"
#include "filesystem"
#include "ssvio/camera.hpp"
#include "ssvio/frontend.hpp"
#include "ssvio/orbextractor.hpp"
#include "ssvio/backend.hpp"
#include "ssvio/loopclosing.hpp"

namespace ssvio {
class System
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  System() = default;
  explicit System(const std::string &config_file_path);
  ~System() = default;
  std::shared_ptr<ui::PangolinWindow> getViewUi() const { return view_ui_; };
  void GenerateSteroCamera();
  void GenerateORBextractor();
  bool RunStep(const cv::Mat &left_img, const cv::Mat &right_img, const double timestamp);

 private:
  std::shared_ptr<ui::PangolinWindow> view_ui_ = nullptr;
  std::string sys_config_file_path_;
  std::shared_ptr<ssvio::Camera> left_camera_ = nullptr;
  std::shared_ptr<ssvio::Camera> right_camera_ = nullptr;
  std::shared_ptr<FrontEnd> frontend_ = nullptr;
  std::shared_ptr<Map> map_ = nullptr;
  std::shared_ptr<Backend> backend_ = nullptr;
  std::shared_ptr<ORBextractor> orb_extractor_ = nullptr, orb_init_extractor_ = nullptr;
  std::shared_ptr<LoopClosing> loop_closing_ = nullptr;
};
} // namespace ssvio

#endif // SSVIO_SYSTEM_HPP