cmake_minimum_required(VERSION 2.8.3)
project(admittance_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
  controller_interface
  hardware_interface
  pluginlib
  actionlib
  actionlib_msgs
  acg_control_msg
  workspace_trajectory_msgs
  force_torque_sensor
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(admittance_controller src/admittance_controller.cpp)

target_link_libraries(
  admittance_controller
  ${catkin_LIBRARIES}
)
