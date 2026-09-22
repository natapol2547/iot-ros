"""Start the LSM9DS1 IMU and the Madgwick orientation filter.

    ros2 launch iot_robot_bringup imu.launch.py
    ros2 launch iot_robot_bringup imu.launch.py params_file:=/path/to/imu.yaml

Publishes imu/data_raw and imu/mag (lsm9ds1_node) and imu/data with the orientation
(imu_filter_madgwick), all in imu_link. Runs on its own or included from robot.launch.py.
The TF from base_link to imu_link comes from robot_state_publisher, which this file does
not start. Parameters and their reasons are in config/imu.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("iot_robot_bringup"), "config", "imu.yaml"),
            description="Parameters of lsm9ds1 and imu_filter"),
        # A missing or failing IMU does not stop the node, it retries by itself; respawn
        # only covers a crash
        Node(
            package="iot_robot_drivers",
            executable="lsm9ds1_node",
            name="lsm9ds1",
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        ),
        # Subscribes to imu/data_raw (and imu/mag with use_mag), publishes imu/data
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter",
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
