cmake_minimum_required(VERSION 3.8)
project(av_integration_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(av_common REQUIRED)
  find_package(av_perception REQUIRED)
  find_package(av_localization REQUIRED)
  find_package(av_planning REQUIRED)
  find_package(av_control REQUIRED)

  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  # Each core package exports its algorithm library + include dirs; pulling them
  # in via ament_target_dependencies links the cores and their headers.
  ament_target_dependencies(test_pipeline
    av_common av_perception av_localization av_planning av_control)
endif()

ament_package()
