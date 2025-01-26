#ifndef DATASET_HPP
#define DATASET_HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "camera.hpp"
#include "frame.hpp"

class Dataset {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Dataset>;

  Dataset(const std::string &dataset_path) : dataset_path_(dataset_path) {}

  bool Init();

  Frame::Ptr NextFrame();

  Camera::Ptr GetCamera(int camera_id) const { return cameras_.at(camera_id); }

private:
  std::string dataset_path_;
  int current_image_index_ = 0;

  std::vector<Camera::Ptr> cameras_;
};

#endif // !DATASET_HPP
