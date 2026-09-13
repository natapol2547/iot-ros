cmake_minimum_required(VERSION 3.16)
project(iot_robot_mujoco)

find_package(ament_cmake REQUIRED)

install(DIRECTORY mjcf
        DESTINATION share/${PROJECT_NAME})

ament_package()
