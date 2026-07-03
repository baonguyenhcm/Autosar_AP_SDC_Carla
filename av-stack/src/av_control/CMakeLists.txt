cmake_minimum_required(VERSION 3.8)
project(av_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(av_common REQUIRED)
find_package(av_msgs REQUIRED)

add_library(controller_core src/controller_core.cpp)
target_include_directories(controller_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(controller_core PUBLIC cxx_std_17)
ament_target_dependencies(controller_core av_common)

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node controller_core)
ament_target_dependencies(control_node rclcpp av_msgs av_common)
rosidl_get_typesupport_target(cpp_typesupport_target av_msgs rosidl_typesupport_cpp)
target_link_libraries(control_node "${cpp_typesupport_target}")

install(TARGETS controller_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_core test/test_controller_core.cpp)
  target_link_libraries(test_controller_core controller_core)
  ament_target_dependencies(test_controller_core av_common)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(av_common)

ament_package()
