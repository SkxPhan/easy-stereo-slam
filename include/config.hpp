#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <memory>
#include <opencv2/core/persistence.hpp>
#include <string>

class Config {
public:
  ~Config();
  static bool SetParameterFile(const std::string &filename);
  template <typename T> static T Get(const std::string &key) {
    return T(Config::config_->file_[key]);
  }

private:
  Config() = default;
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
};

#endif // CONFIG_HPP
