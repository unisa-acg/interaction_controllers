cmake_minimum_required(VERSION 2.8.3)
project(force_torque_sensor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES force_torque_sensor mock_force_torque_sensor gazebo_force_torque_sensor
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(force_torque_sensor src/force_torque_sensor.cpp)
add_library(mock_force_torque_sensor src/mock_force_torque_sensor.cpp)
add_library(gazebo_force_torque_sensor src/gazebo_force_torque_sensor.cpp)

add_executable(demo_force_torque_sensor_read demo/demo_force_torque_sensor_read.cpp)

target_link_libraries(
  mock_force_torque_sensor
  force_torque_sensor
  ${catkin_LIBRARIES}
)

target_link_libraries(
  gazebo_force_torque_sensor
  force_torque_sensor
  ${catkin_LIBRARIES}
)

target_link_libraries(
  demo_force_torque_sensor_read
  force_torque_sensor
  ${catkin_LIBRARIES}
)
