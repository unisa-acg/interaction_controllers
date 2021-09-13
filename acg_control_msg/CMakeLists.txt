cmake_minimum_required(VERSION 2.8.3)
project(acg_control_msg)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  workspace_trajectory_msgs
)

add_action_files(
  DIRECTORY action
  FILES FollowWorkspaceTrajectory.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs workspace_trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs workspace_trajectory_msgs
)