#ifndef VISUAL_ODOMETRY_HPP
#define VISUAL_ODOMETRY_HPP

#include <memory>
#include <string>

#include <Eigen/Core>

#include "backend.hpp"
#include "dataset.hpp"
#include "frontend.hpp"

class VisualOdometry {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<VisualOdometry>;

  VisualOdometry(std::string &config_path) : config_file_path_(config_path) {};

  bool Init();
  void Run();
  bool Step();

  Frontend::Status GetStatus() const { return frontend_->GetStatus(); }

private:
  bool inited_ = false;
  std::string config_file_path_;

  Frontend::Ptr frontend_;
  Backend::Ptr backend_;
  Map::Ptr map_;
  Viewer::Ptr viewer_;

  Dataset::Ptr dataset_;
};

#endif // VISUAL_ODOMETRY_HPP
