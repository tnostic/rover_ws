cmake_minimum_required(VERSION 3.10)
project(rover_one)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(TARGETS
  gps_waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
