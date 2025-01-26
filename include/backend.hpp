#ifndef BACKEND_HPP
#define BACKEND_HPP

#include <condition_variable>
#include <memory>
#include <thread>

#include <Eigen/Core>

#include "camera.hpp"
#include "map.hpp"

class Backend {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Backend>;

  Backend();

  void SetCameras(Camera::Ptr left, Camera::Ptr right) {
    cam_left_ = left;
    cam_right_ = right;
  }

  void SetMap(Map::Ptr map) { map_ = map; }
  void UpdateMap();
  void Stop();

private:
  std::shared_ptr<Map> map_;
  std::thread backend_thread_;
  std::mutex data_mutex_;

  std::condition_variable map_update_;
  std::atomic<bool> backend_running_;

  Camera::Ptr cam_left_;
  Camera::Ptr cam_right_;

  void BackendLoop();
  void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);
};

#endif // BACKEND_HPP
