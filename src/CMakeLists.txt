add_library(myslam SHARED
  camera.cpp
  frame.cpp
  mappoint.cpp
  map.cpp
  frontend.cpp
  backend.cpp
  viewer.cpp
  dataset.cpp
  config.cpp
  visual_odometry.cpp)
target_include_directories(myslam PUBLIC ${CMAKE_SOURCE_DIR}/include ${g2o_SOURCE_DIR} ${g2o_BINARY_DIR})
target_link_libraries(myslam PUBLIC Eigen3::Eigen ${OpenCV_LIBS} Sophus::Sophus glog::glog ${Pangolin_LIBRARIES} g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)
target_compile_features(myslam PUBLIC cxx_std_20)

