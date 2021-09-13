cmake_minimum_required(VERSION 2.8.3)

project(follow_workspace_trajectory_action_client)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  rosbag
  workspace_trajectory
  acg_control_msg
)

catkin_package()

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(follow_workspace_trajectory_action_client src/follow_workspace_trajectory_action_client.cpp)

target_link_libraries(follow_workspace_trajectory_action_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})

