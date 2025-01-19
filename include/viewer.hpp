#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>

#include "frame.hpp"
#include "map.hpp"

class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Viewer>;

  Viewer();

  void SetMap(Map::Ptr map) { map_ = map; }
  void UpdateMap();
  void AddCurrentFrame(Frame::Ptr current_frame);
  void Close();

private:
  Map::Ptr map_;
  Frame::Ptr current_frame_;

  std::thread viewer_thread_;
  bool viewer_running_ = true;

  std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
  std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
  bool map_updated_ = false;

  std::mutex viewer_data_mutex_;

  void ThreadLoop();
  void DrawFrame(Frame::Ptr frame, const float *color);
  void DrawMapPoints();
  void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);
  cv::Mat PlotFrameImage();
};

#endif // VIEWER_HPP
