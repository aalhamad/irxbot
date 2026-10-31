cmake_minimum_required(VERSION 3.8)
project(irx_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(motor_driver_node src/motor_driver_node.cpp)

target_include_directories(motor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(motor_driver_node PUBLIC cxx_std_17)

ament_target_dependencies(motor_driver_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros)

add_executable(esp32_telemetry_node src/esp32_telemetry_node.cpp)

target_compile_features(esp32_telemetry_node PUBLIC cxx_std_17)

ament_target_dependencies(esp32_telemetry_node
  rclcpp
  sensor_msgs
  diagnostic_msgs)

install(TARGETS motor_driver_node esp32_telemetry_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
