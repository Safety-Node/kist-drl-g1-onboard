cmake_minimum_required(VERSION 3.8)
project(camera_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_relay src/camera_relay_node.cpp)
target_compile_features(camera_relay PUBLIC cxx_std_17)
ament_target_dependencies(camera_relay rclcpp sensor_msgs)

install(TARGETS camera_relay DESTINATION lib/${PROJECT_NAME})

ament_package()
