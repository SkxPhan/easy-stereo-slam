#ifndef FRONTEND_HPP

#include <memory>

#include <Eigen/Core>
#include <opencv2/features2d.hpp>
#include <sophus/se3.hpp>

#include "camera.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "viewer.hpp"

class Backend;
class Viewer;

class Frontend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Frontend>;

  enum Status { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

  Frontend();

  bool AddFrame(Frame::Ptr frame);
  void SetMap(Map::Ptr map) { map_ = map; }
  void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
  void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

  Status GetStatus() const { return status_; }

  void SetCameras(Camera::Ptr left, Camera::Ptr right) {
    camera_left_ = left;
    camera_right_ = right;
  }

private:
  Status status_ = INITING;

  Frame::Ptr current_frame_;
  Frame::Ptr last_frame_;
  Camera::Ptr camera_left_;
  Camera::Ptr camera_right_;

  Map::Ptr map_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<Viewer> viewer_;

  Sophus::SE3d relative_motion_;

  int tracking_inliers_ = 0;

  // params
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  // utilities
  cv::Ptr<cv::GFTTDetector> gftt_;

  bool Track();
  bool Reset();
  int TrackLastFrame();
  int EstimateCurrentPose();
  bool InsertKeyframe();
  bool StereoInit();
  int DetectFeatures();
  int FindFeaturesInRight();
  bool BuildInitMap();
  int TriangulateNewPoints();
  void SetObservationsForKeyFrame();
};

#endif // FRONTEND_HPP
