cmake_minimum_required(VERSION 3.16)
project(iot_robot_description)

find_package(ament_cmake REQUIRED)

# Copy these directories into install/share/iot_robot_description/ so that
# package:// URIs and $(find-pkg-share ...) can resolve them at runtime.
install(DIRECTORY urdf meshes launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
