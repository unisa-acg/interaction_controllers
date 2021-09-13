# follow_workspace_trajectory_action_client

This package implements an Action Client over the FollowWorkspaceTrajectory action. It can be used with any Action Server that expects the same message.

In order to use this package, you have to build it using the catkin build command:

```bash
cd /path/to/workspace
catkin build follow_workspace_trajectory_action_client
```

Note that the `workspace_trajectory_msgs` package and the `workspace_trajectory` package must be present in your workspace.

The trajectory to be sent as goal to the Action Server must be in the '.traj' format.  The execution is monitored and the feedback collected in three trajectories, corresponding to **desired**, **actual** and **error**. They are all written together in the same output bagfile.

The preferred usage of this node is within a launch file. In fact, before the `follow_workspace_trajectory_action_client` node can be called, a set of parameters must be present on the parameter server:

* **`action_topic`**: the topic used by the workspace trajectory action server, whose name is used to namespace `<action_topic>/goal`, `<action_topic>/result`, `<action_topic>/feedback`, etc;

* **`fraction_feedback_messages_to_save`**: the fraction of feedback messages to save in the output bagfile, assumed to be between 1 and 100 (refer to `Known issues` section for further information).

The filename corresponding to the input file is rather passed to the node executable as an argument while the output bagfile is placed in the `output` folder of this package, i.e.

```bash
roslaunch follow_workspace_trajectory_action_client follow_workspace_trajectory_action_client.launch filename:=ur10_gazebo_180s_50N_ramp.traj
```

The `trajectories` folder already contains some trajectories:

* **ur10\_\*** and **smartsix\_\*** indicates trajectories for the UR10 and the COMAU Smart-Six robots;
* **\*\_gazebo\_\*** are all trajectories for the execution in Gazebo with both admittance and direct force controller;
* **\*\_XXXs\_\*** is the duration of the trajectory expressed in seconds;
* **\*\_XXXN\_\*** is the max force along the z axis achieved during the trajectory execution expressed in Newton;
* **\*\_const** indicates that the force reference is constant along the whole trajectory;
* **\*\_ramp** indicates that the force profile is a ramp starting from zero to the max force value.

Provided trajectories are all straight lines moving along the x axis, from left to right with respect to the initial position.

Depending on the adopted controller it could be preferable to manually bring the robot near to the initial position and orientation of these trajectories before executing them.

To extract details of the executed trajectories, you may use MATLAB custom messages. Further information about custom messages in MATLAB can be found at [this page](https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html).

## Known issues

* The error

```bash
Terminate called after throwing an instance of 'ros::serialization::StreamOverrunException'
what():  Buffer Overrun
```

may happen under certain circumstances. These conditions depend on the current state of the machine you are using, which may not be able to handle the exchange of feedback messages between client and server. This could lead to missing a lot of feedback messages or raising this error. In order to avoid this issue:

* clean your RAM before communication;

* decrease the number of feedback messages to save using the `fraction_feedback_messages_to_save` parameter present on the parameter server.

Note that MATLAB might fail loading large bagfiles.
