cmake_minimum_required(VERSION 3.20)
project(easy-stereo-slam VERSION 0.1.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Glog REQUIRED)

add_subdirectory(src)
