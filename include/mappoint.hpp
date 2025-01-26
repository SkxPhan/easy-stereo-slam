#ifndef MAPPOINT_HPP
#define MAPPOINT_HPP

#include <list>
#include <memory>
#include <mutex>

#include <Eigen/Core>

#include "feature.hpp"

struct MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<MapPoint>;

  std::mutex data_mutex_;
  unsigned long id_ = 0;
  bool is_outlier_ = false;
  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
  int observed_times_ = 0;
  std::list<std::weak_ptr<Feature>> observations_;

  MapPoint() = default;

  MapPoint(long id, Eigen::Vector3d position);

  Eigen::Vector3d Pos() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return pos_;
  }

  void SetPos(const Eigen::Vector3d &pos) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    pos_ = pos;
  }

  void AddObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
  }

  void RemoveObservation(std::shared_ptr<Feature> feature);

  std::list<std::weak_ptr<Feature>> GetObs() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return observations_;
  }

  static MapPoint::Ptr CreateNewMappoint();
};

#endif // MAPPOINT_HPP
