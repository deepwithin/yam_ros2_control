cmake_minimum_required(VERSION 3.16)
project(i2rt_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(leader_follower_node src/leader_follower_node.cpp)
target_compile_features(leader_follower_node PUBLIC cxx_std_17)
ament_target_dependencies(leader_follower_node
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)

# Debug tool only - see its file comment. Not part of normal operation.
add_executable(command_pose_node src/command_pose_node.cpp)
target_compile_features(command_pose_node PUBLIC cxx_std_17)
ament_target_dependencies(command_pose_node
  rclcpp
  trajectory_msgs
)

install(
  TARGETS leader_follower_node command_pose_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
