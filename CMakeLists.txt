cmake_minimum_required(VERSION 3.20)
project(easy-stereo-slam VERSION 0.1.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include(FetchContent)

FetchContent_Declare(
    Pangolin
    GIT_REPOSITORY https://github.com/stevenlovegrove/Pangolin.git
    GIT_TAG v0.9.2
)
FetchContent_MakeAvailable(Pangolin)

FetchContent_Declare(
    g2o
    GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
    GIT_TAG 20241228_git
)
FetchContent_MakeAvailable(g2o)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(glog REQUIRED)

FetchContent_GetProperties(g2o)
# if(g2o_POPULATED)
#   message(STATUS "g2o source directory: ${g2o_SOURCE_DIR}")
#   message(STATUS "g2o binary directory: ${g2o_BINARY_DIR}")
#   message(STATUS "g2o libraries linked: ${g2o_LIBRARIES}")
# endif()

add_subdirectory(src)
add_subdirectory(app)
