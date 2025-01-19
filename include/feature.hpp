#ifndef FEATURE_HPP
#define FEATURE_HPP

#include <memory>

#include <Eigen/Core>
#include <opencv2/features2d.hpp>

struct Frame;
struct MapPoint;

struct Feature {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<Feature>;

  std::weak_ptr<Frame> frame_;
  cv::KeyPoint position_;
  std::weak_ptr<MapPoint> map_point_;

  bool is_outlier_ = false;
  bool is_on_left_image_ = true;

public:
  Feature() = default;

  Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
      : frame_(frame), position_(kp) {}
};

#endif // FEATURE_HPP
