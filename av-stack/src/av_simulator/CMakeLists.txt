cmake_minimum_required(VERSION 3.8)
project(av_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(av_common REQUIRED)
find_package(av_msgs REQUIRED)

add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(simulator_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs av_msgs av_common)
rosidl_get_typesupport_target(cpp_typesupport_target av_msgs rosidl_typesupport_cpp)
target_link_libraries(simulator_node "${cpp_typesupport_target}")

install(TARGETS simulator_node DESTINATION lib/${PROJECT_NAME})

ament_package()
