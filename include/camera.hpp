#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <memory>

#include <Eigen/Core>
#include <sophus/se3.hpp>

class Camera {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Camera>;

  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0, baseline_ = 0.0;
  Sophus::SE3d pose_;
  Sophus::SE3d pose_inv_;

  Camera() = default;

  Camera(double fx, double fy, double cx, double cy, double baseline,
         const Sophus::SE3d &pose)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
    pose_inv_ = pose_.inverse();
  }

  Sophus::SE3d pose() const { return pose_; }

  Eigen::Matrix3d K() const {
    Eigen::Matrix3d k;
    k << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
    return k;
  }

  Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w,
                               const Sophus::SE3d &T_c_w) const;
  Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c,
                               const Sophus::SE3d &T_c_w) const;
  Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c) const;
  Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p,
                               double depth = 1.0) const;
  Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p,
                              const Sophus::SE3d &T_c_w,
                              double depth = 1.0) const;
  Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w,
                              const Sophus::SE3d &T_c_w) const;
};

#endif // CAMERA_HPP
