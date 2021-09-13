# workspace_trajectory_msgs

This package includes the message definition corresponding to the `WorkspaceTrajectory` class from the `workspace_trajectory` package. It is simply made by a vector of waypoints (`geometry_msgs/Pose` messages), vector of wrenches (`geometry_msgs/Wrench` messages) and the relating timestamps.

In the future, the `WorkspaceTrajectoryPoint` will be defined and the `WorkspaceTrajectory` message will be made of a vector of workspace trajectory points.