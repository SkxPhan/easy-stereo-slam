#include "frontend.hpp"

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "algorithm.hpp"
#include "backend.hpp"
#include "config.hpp"
#include "g2o_types.hpp"

Frontend::Frontend() {
  num_features_init_ = Config::Get<int>("num_features_init");
  num_features_ = Config::Get<int>("num_features");
  gftt_ = cv::GFTTDetector::create(num_features_, 0.01, 20);
}

bool Frontend::AddFrame(Frame::Ptr frame) {
  current_frame_ = frame;

  switch (status_) {
  case INITING:
    StereoInit();
    break;
  case TRACKING_GOOD:
  case TRACKING_BAD:
    Track();
    break;
  case LOST:
    Reset();
    break;
  }

  last_frame_ = current_frame_;
  return true;
}

bool Frontend::Track() {
  if (last_frame_) {
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
  }

  int num_track_last = TrackLastFrame();
  tracking_inliers_ = EstimateCurrentPose();

  if (tracking_inliers_ > num_features_tracking_) {
    status_ = TRACKING_GOOD;
  } else if (tracking_inliers_ > num_features_tracking_bad_) {
    status_ = TRACKING_BAD;
  } else {
    status_ = LOST;
  }

  InsertKeyframe();
  relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

  if (viewer_) {
    viewer_->AddCurrentFrame(current_frame_);
  }

  return true;
}

bool Frontend::InsertKeyframe() {
  if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
    return false;
  }

  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);

  LOG(INFO) << "Set frame " << current_frame_->id_ << "as keyframe "
            << current_frame_->keyframe_id_;

  SetObservationsForKeyFrame();
  DetectFeatures();

  FindFeaturesInRight();
  TriangulateNewPoints();
  backend_->UpdateMap();

  if (viewer_)
    viewer_->UpdateMap();

  return true;
}

void Frontend::SetObservationsForKeyFrame() {
  for (const auto &feat : current_frame_->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->AddObservation(feat);
    }
  }
}

int Frontend::TriangulateNewPoints() {
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  Sophus::SE3d current_pose_Twc = current_frame_->Pose().inverse();
  int cnt_triangulated_pts = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.expired() &&
        current_frame_->features_right_[i]) {
      auto &feature_left = current_frame_->features_left_[i];
      auto &feature_right = current_frame_->features_right_[i];
      std::vector<Eigen::Vector3d> points{
          camera_left_->pixel2camera(
              myslam::algo::toVec2(feature_left->position_.pt)),
          camera_right_->pixel2camera(
              myslam::algo::toVec2(feature_right->position_.pt))};
      Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

      if (myslam::algo::triangulation(poses, points, pworld) && pworld[2] > 0) {
        auto new_map_point = MapPoint::CreateNewMappoint();
        pworld = current_pose_Twc * pworld;
        new_map_point->SetPos(pworld);
        new_map_point->AddObservation(feature_left);
        new_map_point->AddObservation(feature_right);

        feature_left->map_point_ = new_map_point;
        feature_right->map_point_ = new_map_point;
        map_->InsertMapPoint(new_map_point);
        cnt_triangulated_pts++;
      }
    }
  }
  LOG(INFO) << "New landmarkds: " << cnt_triangulated_pts;
  return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose() {
  using BlockSolverType = g2o::BlockSolver_6_3;
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // vertex
  auto vertex_pose = new VertexPose(); // camera vertex_pose
  vertex_pose->setId(0);
  vertex_pose->setEstimate(current_frame_->Pose());
  optimizer.addVertex(vertex_pose);

  // K
  Eigen::Matrix3d K = camera_left_->K();

  // edges
  int index = 1;
  std::vector<EdgeProjectionPoseOnly *> edges;
  std::vector<Feature::Ptr> features;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    auto mp = current_frame_->features_left_[i]->map_point_.lock();
    if (mp) {
      features.push_back(current_frame_->features_left_[i]);
      auto edge = new EdgeProjectionPoseOnly(mp->pos_, K);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(myslam::algo::toVec2(
          current_frame_->features_left_[i]->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(
          std::make_unique<g2o::RobustKernelHuber>().release());
      edges.push_back(edge);
      optimizer.addEdge(edge);
      index++;
    }
  }

  // estimate the Pose the determine the outliers
  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iteration = 0; iteration < 4; ++iteration) {
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cnt_outlier = 0;

    // count the outliers
    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->is_outlier_) {
        e->computeError();
      }
      if (e->chi2() > chi2_th) {
        features[i]->is_outlier_ = true;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->is_outlier_ = false;
        e->setLevel(0);
      };

      if (iteration == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }

  LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
            << features.size() - cnt_outlier;
  // Set pose and outlier
  current_frame_->SetPose(vertex_pose->estimate());

  LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

  for (auto &feat : features) {
    if (feat->is_outlier_) {
      feat->map_point_.reset();
      feat->is_outlier_ = false; // maybe we can still use it in future
    }
  }
  return static_cast<int>(features.size()) - cnt_outlier;
}

int Frontend::TrackLastFrame() {
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_last, kps_current;
  for (auto &kp : last_frame_->features_left_) {
    if (kp->map_point_.lock()) {
      // use project point
      auto mp = kp->map_point_.lock();
      auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(cv::Point2f(px[0], px[1]));
    } else {
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current,
      status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_current[i], 7);
      Feature::Ptr feature(new Feature(current_frame_, kp));
      feature->map_point_ = last_frame_->features_left_[i]->map_point_;
      current_frame_->features_left_.push_back(feature);
      num_good_pts++;
    }
  }

  LOG(INFO) << "Find " << num_good_pts << " in the last image.";
  return num_good_pts;
}

bool Frontend::StereoInit() {
  int num_features_left = DetectFeatures();
  int num_coor_features = FindFeaturesInRight();
  if (num_coor_features < num_features_init_) {
    return false;
  }

  bool build_map_success = BuildInitMap();
  if (build_map_success) {
    status_ = TRACKING_GOOD;
    if (viewer_) {
      viewer_->AddCurrentFrame(current_frame_);
      viewer_->UpdateMap();
    }
    return true;
  }
  return false;
}

int Frontend::DetectFeatures() {
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto &feat : current_frame_->features_left_) {
    cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                  feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(current_frame_->left_img_, keypoints, mask);
  int cnt_detected = 0;
  for (auto &kp : keypoints) {
    current_frame_->features_left_.push_back(
        Feature::Ptr(new Feature(current_frame_, kp)));
    cnt_detected++;
  }

  LOG(INFO) << "Detect " << cnt_detected << " new features";
  return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_left, kps_right;
  for (auto &kp : current_frame_->features_left_) {
    kps_left.push_back(kp->position_.pt);
    auto mp = kp->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_right.push_back(cv::Point2f(px[0], px[1]));
    } else {
      // use same pixel in left iamge
      kps_right.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      current_frame_->left_img_, current_frame_->right_img_, kps_left,
      kps_right, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_right[i], 7);
      Feature::Ptr feat(new Feature(current_frame_, kp));
      feat->is_on_left_image_ = false;
      current_frame_->features_right_.push_back(feat);
      num_good_pts++;
    } else {
      current_frame_->features_right_.push_back(nullptr);
    }
  }
  LOG(INFO) << "Find " << num_good_pts << " in the right image.";
  return num_good_pts;
}

bool Frontend::BuildInitMap() {
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  size_t cnt_init_landmarks = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_right_[i] == nullptr)
      continue;
    // create map point from triangulation
    std::vector<Eigen::Vector3d> points{
        camera_left_->pixel2camera(
            Eigen::Vector2d(current_frame_->features_left_[i]->position_.pt.x,
                            current_frame_->features_left_[i]->position_.pt.y)),
        camera_right_->pixel2camera(Eigen::Vector2d(
            current_frame_->features_right_[i]->position_.pt.x,
            current_frame_->features_right_[i]->position_.pt.y))};
    Eigen::Vector3d pworld = Eigen::Vector3d::Zero();

    if (myslam::algo::triangulation(poses, points, pworld) && pworld[2] > 0) {
      auto new_map_point = MapPoint::CreateNewMappoint();
      new_map_point->SetPos(pworld);
      new_map_point->AddObservation(current_frame_->features_left_[i]);
      new_map_point->AddObservation(current_frame_->features_right_[i]);
      current_frame_->features_left_[i]->map_point_ = new_map_point;
      current_frame_->features_right_[i]->map_point_ = new_map_point;
      cnt_init_landmarks++;
      map_->InsertMapPoint(new_map_point);
    }
  }
  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);
  backend_->UpdateMap();

  LOG(INFO) << "Initial map created with " << cnt_init_landmarks
            << " map points";

  return true;
}

bool Frontend::Reset() {
  LOG(INFO) << "Reset is not implemented. ";
  return true;
}
