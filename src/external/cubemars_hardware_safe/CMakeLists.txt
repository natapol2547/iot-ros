cmake_minimum_required(VERSION 3.16)
project(cubemars_hardware_safe LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(cubemars_hardware REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)

add_library(${PROJECT_NAME} SHARED src/safe_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC cubemars_hardware::cubemars_hardware)
target_link_libraries(${PROJECT_NAME} PRIVATE joint_limits::joint_limits urdf::urdf)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface cubemars_hardware_safe.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(cubemars_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
