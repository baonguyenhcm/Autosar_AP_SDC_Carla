cmake_minimum_required(VERSION 3.8)
project(av_localization)

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

add_library(ekf_localizer src/ekf_localizer.cpp)
target_include_directories(ekf_localizer PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(ekf_localizer PUBLIC cxx_std_17)
ament_target_dependencies(ekf_localizer av_common)

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ekf_localizer)
ament_target_dependencies(localization_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs av_msgs av_common)
rosidl_get_typesupport_target(cpp_typesupport_target av_msgs rosidl_typesupport_cpp)
target_link_libraries(localization_node "${cpp_typesupport_target}")

install(TARGETS ekf_localizer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf_localizer test/test_ekf_localizer.cpp)
  target_link_libraries(test_ekf_localizer ekf_localizer)
  ament_target_dependencies(test_ekf_localizer av_common)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(av_common)

ament_package()
