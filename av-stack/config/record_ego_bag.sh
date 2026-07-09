#!/usr/bin/env bash
# Record the ego-motion verification bag. Run INSIDE autoware-dev, stop with Ctrl-C
# (the bag is only readable after a clean stop — metadata.yaml is written on shutdown).
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="file:///home/nguyennqb/av-stack-config/cyclonedds-local.xml"
OUT=${1:-/tmp/ego_motion_bag_$(date +%H%M%S)}
exec ros2 bag record -o "$OUT" \
  /carla/ego_vehicle/odometry \
  /carla/ego_vehicle/lidar \
  /carla/ego_vehicle/imu \
  /carla/ego_vehicle/vehicle_control_cmd \
  /tf /clock
