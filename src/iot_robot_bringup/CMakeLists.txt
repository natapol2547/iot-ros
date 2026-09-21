cmake_minimum_required(VERSION 3.16)
project(iot_robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
