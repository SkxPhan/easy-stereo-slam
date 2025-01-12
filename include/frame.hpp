#ifndef FRAME_HPP
#define FRAME_HPP

#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include "feature.hpp"

struct Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<Frame>;

  unsigned long id_ = 0;
  unsigned long keyframe_id_ = 0;
  bool is_keyframe_ = false;
  double timestamp_;
  Sophus::SE3d pose_;
  std::mutex pose_mutex_;
  cv::Mat left_img_, right_img_;

  std::vector<std::shared_ptr<Feature>> features_left_;
  std::vector<std::shared_ptr<Feature>> features_right_;

public:
  Frame() {};

  Frame(long id, double timestamp, const Sophus::SE3d &pose,
        const cv::Mat &left, const cv::Mat &right);

  Sophus::SE3d pose() {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_;
  }

  void setPose(const Sophus::SE3d &pose) {
    std::unique_lock<std::mutex> lock(pose_mutex_);
    pose_ = pose;
  }

  void setKeyFrame();

  static std::shared_ptr<Frame> createFrame();
};

#endif // FRAME_HPP
