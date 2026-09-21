cmake_minimum_required(VERSION 3.16)
project(iot_robot_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch mjcf urdf
        DESTINATION share/${PROJECT_NAME})

# Sim-only helper nodes, run with `ros2 run iot_robot_mujoco <script>.py`
install(PROGRAMS scripts/scan_to_range.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
