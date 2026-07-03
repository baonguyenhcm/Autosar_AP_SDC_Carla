# av-stack — AUTOSAR Adaptive Platform build.
#   Host build (default):     cores + av_ap + host tests (no lwrcl, no ROS).
#   AP target build:          -DAP_BACKEND=adaptive-autosar builds the Adaptive
#                             Applications on ara::com (Adaptive-AUTOSAR, SOME/IP) and the
#                             CARLA gateway on ara::com + lwrcl (CycloneDDS, ROS 2 wire),
#                             and defines AP_HAVE_ARA.
cmake_minimum_required(VERSION 3.16)
project(av_stack_ap CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)
set(AP_BACKEND "off" CACHE STRING "off | adaptive-autosar")

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(AV_INCLUDES
  ${SRC}/av_common/include ${SRC}/av_ap/include
  ${SRC}/av_perception/include ${SRC}/av_localization/include
  ${SRC}/av_planning/include ${SRC}/av_control/include)

# ROS/middleware-free algorithm cores (always built).
add_library(av_cores STATIC
  ${SRC}/av_perception/src/perception_core.cpp
  ${SRC}/av_localization/src/ekf_localizer.cpp
  ${SRC}/av_planning/src/planner_core.cpp
  ${SRC}/av_control/src/controller_core.cpp)
target_include_directories(av_cores PUBLIC ${AV_INCLUDES})

# Host tests (no lwrcl needed).
enable_testing()
add_executable(test_soa_pipeline ${SRC}/av_integration_tests/test/test_soa_pipeline.cpp)
target_include_directories(test_soa_pipeline PRIVATE ${AV_INCLUDES})
target_link_libraries(test_soa_pipeline av_cores)
add_test(NAME test_soa_pipeline COMMAND test_soa_pipeline)

add_executable(test_ap_integration ${SRC}/av_ap/test/test_ap_integration.cpp)
target_include_directories(test_ap_integration PRIVATE ${AV_INCLUDES})
target_link_libraries(test_ap_integration av_cores)
add_test(NAME test_ap_integration COMMAND test_ap_integration)

# AP target: AAs on ara::com (SOME/IP); the gateway ALSO links lwrcl (CycloneDDS).
if(AP_BACKEND STREQUAL "adaptive-autosar")
  message(STATUS "AP target: AAs on ara::com/SOME/IP; gateway also on lwrcl/CycloneDDS")
  add_compile_definitions(AP_HAVE_ARA)

  # --- AP Functional Clusters from Adaptive-AUTOSAR (exported AdaptiveAutosarAP::ara_*) ---
  # Needs /opt/autosar-ap on CMAKE_PREFIX_PATH (set in build_ap.sh).
  find_package(AdaptiveAutosarAP REQUIRED)
  set(ARA_LIBS
    AdaptiveAutosarAP::ara_core AdaptiveAutosarAP::ara_com
    AdaptiveAutosarAP::ara_log  AdaptiveAutosarAP::ara_exec
    AdaptiveAutosarAP::ara_phm)

  # --- lwrcl, CycloneDDS backend, for the GATEWAY's ROS 2 side ONLY ---
  # lwrcl installs a bare liblwrcl.so + headers (NO CMake package), so import it by hand
  # from the cyclonedds prefix. This is the ROS 2-wire backend that interoperates with
  # zenoh-bridge-ros2dds. Do NOT use the adaptive-autosar lwrcl (/opt/autosar-ap-libs) for
  # the gateway — its ara::com/AUTOSAR topic mapping is not ROS 2 wire-compatible.
  set(LWRCL_CYCLONEDDS_PREFIX "/opt/cyclonedds-libs"
      CACHE PATH "lwrcl (cyclonedds backend) install prefix")
  add_library(lwrcl_ros2 SHARED IMPORTED)
  set_target_properties(lwrcl_ros2 PROPERTIES
    IMPORTED_LOCATION "${LWRCL_CYCLONEDDS_PREFIX}/lib/liblwrcl.so"
    INTERFACE_INCLUDE_DIRECTORIES "${LWRCL_CYCLONEDDS_PREFIX}/include")
  # NOTE: the gateway also needs the ROS 2 message headers/libs from
  # `build_data_types.sh cyclonedds` (sensor_msgs, nav_msgs, carla_msgs). Add their
  # include/lib prefix to lwrcl_ros2 (or a second imported target) once confirmed on target.

  # Internal Adaptive Applications: ara::com only (no lwrcl).
  foreach(app localization perception planning control)
    add_executable(${app}_app ${SRC}/av_${app}/src/${app}_app_main.cpp)
    target_include_directories(${app}_app PRIVATE ${AV_INCLUDES})
    target_link_libraries(${app}_app av_cores ${ARA_LIBS})
  endforeach()

  add_executable(safe_stop_app ${SRC}/av_control/src/safe_stop_app_main.cpp)
  target_include_directories(safe_stop_app PRIVATE ${AV_INCLUDES})
  target_link_libraries(safe_stop_app av_cores ${ARA_LIBS})

  # Gateway: ara::com (AP side -> AAs) + lwrcl/CycloneDDS (ROS 2 side -> the bridge).
  add_executable(carla_gateway ${SRC}/av_gateway/src/carla_gateway_node.cpp)
  target_include_directories(carla_gateway PRIVATE ${AV_INCLUDES})
  target_link_libraries(carla_gateway av_cores ${ARA_LIBS} lwrcl_ros2)

  install(TARGETS localization_app perception_app planning_app control_app
                  safe_stop_app carla_gateway RUNTIME DESTINATION bin)
  install(DIRECTORY config/ DESTINATION share/av_stack/config)
endif()
