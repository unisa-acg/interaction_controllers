# Interaction controllers

This repository is a collection of ROS packages needed to perform interaction control on both simulated and real robots. Please cite:

> Storiale, F.; Ferrentino, E.; Chiacchio, P. Robot-Agnostic Interaction Controllers Based on ROS. Appl. Sci. 2022, 12, 3949. https://doi.org/10.3390/app12083949

The repository is structured as follows:

* the `ros_control_controllers` package contains the implementation of the direct force controller and admittance controller;
* the `force_torque_sensor` package contains the library to handle the communication with real and simulated force/torque sensors;
* the `workspace_trajectory_msgs` and `workspace_trajectory` packages contain the definition of a workspace trajectory message and a library to manage it;
* the `acg_control_msgs` package defines the `FollowWorkspaceTrajectory` message needed for the transmission of a workspace trajectory through the action interface;
* `follow_workspace_trajectory_action_client` package implements an Action Client over the `FollowWorkspaceTrajectory` action.
* the `smartsix` container provides description, configuration and start up files for the Comau Smart-Six robot;
* the `ur10_acg_configuration` container provides configuration and start up files for the Universal Robot's UR10 robot.

Refer to the Readmes of the single packages for further information.
