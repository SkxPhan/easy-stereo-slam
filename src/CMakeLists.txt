add_library(myslam SHARED
  frame.cpp
  map.cpp
  mappoint.cpp)
target_include_directories(myslam PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(myslam PRIVATE Eigen3::Eigen ${OpenCV_LIBS} Sophus::Sophus ${GLOG_LIBRARIES})
target_compile_features(myslam PRIVATE cxx_std_20)
