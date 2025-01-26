#include "config.hpp"

#include <glog/logging.h>
#include <memory>

bool Config::SetParameterFile(const std::string &filename) {
  if (!config_) {
    config_ = std::make_shared<Config>();
  }

  config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
  if (!config_->file_.isOpened()) {
    LOG(ERROR) << "Parameter file " << filename << " does not exist.";
    config_->file_.release();
    return false;
  }

  return true;
}

Config::~Config() {
  if (file_.isOpened()) {
    file_.release();
  }
}

std::shared_ptr<Config> Config::config_;
