"""Bring up the full av-stack closed loop:

  simulator -> (lidar/imu/gps/speed) -> perception + localization
            -> planning -> control -> (command) -> simulator

Run:  ros2 launch av_bringup av_stack.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Canonical topic names shared across the graph.
T_POINTS = "/sensing/lidar/points"
T_IMU = "/sensing/imu"
T_GPS = "/sensing/gps"
T_SPEED = "/vehicle/wheel_speed"
T_STATE = "/localization/vehicle_state"
T_OBJECTS = "/perception/objects"
T_TRAJ = "/planning/trajectory"
T_CMD = "/control/command"


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("av_bringup"), "config", "params.yaml")

    simulator = Node(
        package="av_simulator", executable="simulator_node", name="simulator_node",
        parameters=[params],
        remappings=[
            ("output/points", T_POINTS),
            ("output/imu", T_IMU),
            ("output/gps", T_GPS),
            ("output/wheel_speed", T_SPEED),
            ("output/ground_truth", "/sim/ground_truth"),
            ("input/command", T_CMD),
        ])

    perception = Node(
        package="av_perception", executable="perception_node", name="perception_node",
        parameters=[params],
        remappings=[
            ("input/points", T_POINTS),
            ("input/vehicle_state", T_STATE),
            ("output/objects", T_OBJECTS),
        ])

    localization = Node(
        package="av_localization", executable="localization_node", name="localization_node",
        parameters=[params],
        remappings=[
            ("input/imu", T_IMU),
            ("input/gps", T_GPS),
            ("input/wheel_speed", T_SPEED),
            ("output/vehicle_state", T_STATE),
            ("output/odometry", "/localization/odometry"),
        ])

    planning = Node(
        package="av_planning", executable="planning_node", name="planning_node",
        parameters=[params],
        remappings=[
            ("input/vehicle_state", T_STATE),
            ("input/objects", T_OBJECTS),
            ("output/trajectory", T_TRAJ),
        ])

    control = Node(
        package="av_control", executable="control_node", name="control_node",
        parameters=[params],
        remappings=[
            ("input/vehicle_state", T_STATE),
            ("input/trajectory", T_TRAJ),
            ("output/command", T_CMD),
        ])

    return LaunchDescription([simulator, perception, localization, planning, control])
