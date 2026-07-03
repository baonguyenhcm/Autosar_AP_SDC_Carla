cmake_minimum_required(VERSION 3.8)
project(av_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(av_common REQUIRED)
find_package(av_msgs REQUIRED)

# --- ROS-free algorithm core ---
add_library(perception_core src/perception_core.cpp)
target_include_directories(perception_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(perception_core PUBLIC cxx_std_17)
ament_target_dependencies(perception_core av_common)

# --- rclcpp node ---
add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node perception_core)
ament_target_dependencies(perception_node rclcpp sensor_msgs av_msgs av_common)
rosidl_get_typesupport_target(cpp_typesupport_target av_msgs rosidl_typesupport_cpp)
target_link_libraries(perception_node "${cpp_typesupport_target}")

install(TARGETS perception_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_core test/test_perception_core.cpp)
  target_link_libraries(test_perception_core perception_core)
  ament_target_dependencies(test_perception_core av_common)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(av_common)

ament_package()
