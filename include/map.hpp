#ifndef MAP_HPP
#define MAP_HPP

#include <memory>
#include <mutex>
#include <unordered_map>

#include <Eigen/Core>

#include "feature.hpp"
#include "frame.hpp"
#include "mappoint.hpp"

class Map {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<Map>;

  using KeyframesType = std::unordered_map<unsigned long, Frame::Ptr>;
  using LandmarksType = std::unordered_map<unsigned long, MapPoint::Ptr>;

  Map() = default;

  void InsertKeyFrame(Frame::Ptr frame);
  void InsertMapPoint(MapPoint::Ptr map_point);

  KeyframesType GetAllKeyFrames() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_keyframes_;
  }

  LandmarksType GetAllMapPoints() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return landmarks_;
  }

  KeyframesType GetActiveKeyFrames() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_keyframes_;
  }

  LandmarksType GetActiveMapPoints() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return active_landmarks_;
  }

  void CleanMap();

private:
  std::mutex data_mutex_;

  KeyframesType keyframes_;
  KeyframesType active_keyframes_;
  LandmarksType landmarks_;
  LandmarksType active_landmarks_;

  Frame::Ptr current_frame_ = nullptr;

  int num_active_keyframes_ = 7;

  void RemoveOldKeyframe();
};

#endif // MAP_HPP
