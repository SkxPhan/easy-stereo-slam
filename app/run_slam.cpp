#include <gflags/gflags.h>
#include <iostream>
#include <memory>
#include <cassert>
#include <filesystem>

#include "visual_odometry.hpp"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char** argv)
{
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::cout << "Loading config file: " << FLAGS_config_file << "\n";

  auto vo = std::make_shared<VisualOdometry>(FLAGS_config_file);
  assert(vo->Init() == true);
  vo->Run();

  return 0;
}
