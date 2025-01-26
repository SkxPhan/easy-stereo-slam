add_executable(run_slam run_slam.cpp)
target_include_directories(run_slam PUBLIC ${CMAKE_SOURCE_DIR}/include ${g2o_SOURCE_DIR} ${g2o_BINARY_DIR})
target_sources(run_slam
  PRIVATE
    ${g2o_SOURCE_DIR}/g2o/stuff/logger.cpp
)
target_link_libraries(run_slam PUBLIC myslam Eigen3::Eigen ${OpenCV_LIBS} Sophus::Sophus glog::glog ${Pangolin_LIBRARIES} g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)
target_compile_features(run_slam PRIVATE cxx_std_20)

