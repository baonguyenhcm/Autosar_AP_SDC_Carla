# Standalone (non-ROS) build of the end-to-end pipeline demo.
# Usage:
#   cmake -S tools -B build_demo && cmake --build build_demo
#   ./build_demo/pipeline_demo
cmake_minimum_required(VERSION 3.10)
project(av_pipeline_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GCC/Clang warning flags; MSVC uses /W4 instead (so a native Windows build
# with cl.exe does not choke on -Wall -Wextra).
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra)
endif()

# Optional: stream ego pose over UDP to a live Unreal Engine viewer.
#   cmake -S tools -B build_demo -DAV_UDP=ON
option(AV_UDP "Stream ego pose over UDP (127.0.0.1:9999) for live UE4 view" OFF)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(pipeline_demo
  pipeline_demo.cpp
  ${SRC_DIR}/av_perception/src/perception_core.cpp
  ${SRC_DIR}/av_localization/src/ekf_localizer.cpp
  ${SRC_DIR}/av_planning/src/planner_core.cpp
  ${SRC_DIR}/av_control/src/controller_core.cpp
)

target_include_directories(pipeline_demo PRIVATE
  ${SRC_DIR}/av_common/include
  ${SRC_DIR}/av_perception/include
  ${SRC_DIR}/av_localization/include
  ${SRC_DIR}/av_planning/include
  ${SRC_DIR}/av_control/include
)

if(AV_UDP)
  target_compile_definitions(pipeline_demo PRIVATE AV_UDP)
  find_package(Threads REQUIRED)
  target_link_libraries(pipeline_demo Threads::Threads)
  if(WIN32)
    target_link_libraries(pipeline_demo ws2_32)
  endif()
endif()
