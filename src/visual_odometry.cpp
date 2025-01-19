#include "visual_odometry.hpp"

#include <chrono>
#include <memory>

#include "config.hpp"
#include <glog/logging.h>
#include <opencv2/videoio.hpp>

bool VisualOdometry::Init() {
  if (!Config::SetParameterFile(config_file_path_)) {
    return false;
  }

  dataset_ =
      std::make_shared<Dataset>(Config::Get<std::string>("datatset_dir"));
  CHECK_EQ(dataset_->Init(), true);

  frontend_ = std::make_shared<Frontend>();
  backend_ = std::make_shared<Backend>();
  map_ = std::make_shared<Map>();
  viewer_ = std::make_shared<Viewer>();

  frontend_->SetBackend(backend_);
  frontend_->SetMap(map_);
  frontend_->SetViewer(viewer_);
  frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  backend_->SetMap(map_);
  backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

  viewer_->SetMap(map_);

  return true;
}

void VisualOdometry::Run() {
  while (1) {
    LOG(INFO) << "VO is running";
    if (!Step()) {
      break;
    }
  }

  backend_->Stop();
  viewer_->Close();

  LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
  Frame::Ptr new_frame = dataset_->NextFrame();
  if (!new_frame)
    return false;

  const auto start_time = std::chrono::steady_clock::now();
  const bool success = frontend_->AddFrame(new_frame);
  const auto end_time = std::chrono::steady_clock::now();
  const auto duration = std::chrono::duration<double>(end_time - start_time);
  LOG(INFO) << "VO cost time: " << duration.count() << " seconds.\n";
  return success;
}
