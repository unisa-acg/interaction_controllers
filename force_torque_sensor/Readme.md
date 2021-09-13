# Force/Torque Sensor Interface

In this documentation we provide a short introduction to the force/torque sensors and related software and hardware architecture.

## Hardware force/torque sensors

The Automatic Control Group at UNISA uses different hardware sensors for their robots. For every sensor/driver (sometimes the same driver can manage more than one sensor), a new implementation of the abstract class `ForceTorqueSensor` is provided.

### Adding a new sensor to the hardware architecture

In order to integrate a new sensor to the hardware architecture, you have to follow these steps:

- Wrap the sensor driver in a class implementing `ForceTorqueSensor`
- Create new yaml file with new sensor parameters
- Write a new launch file that loads the yaml file with the sensor parameters

## Software force/torque sensors

### Mock sensor

A force/torque mock sensor supports testing of components that use F/T sensors, but do not rely on F/T sensor readings.
This mock sensor, in a simple way, returns a random value between 0 and 1 for each reading.

### Gazebo sensor

This sensor is loaded directly into the Gazebo environment through the use of a specific plugin, which publishes force/torque readings on a ROS topic.
Access to sensor data is possible through an implementation of the `ForceTorqueSensor` interface which embeds a subscriber on the same topic.
If you want to listen on the force/torque sensor topic using the `rostopic echo` command, the topic name is `gazebo_ft_sensor`.

## Build and run the ROS package

Clone this package in the source directory of your catkin workspace. If you do not have a catkin worspace you can follow [this guide](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

In the catkin workspace, build the package with

```bash
catkin build force_torque_sensor
```

After you source the workspace, you can run the following demo using the mock sensor:

```bash
roslaunch force_torque_sensor demo_read_mock_sensor.launch
```

Note that to use the Gazebo sensor, you must first run a Gazebo environment loading the sensor. In this package, we provide a demo based on the UR10 robot, so make sure the latest version of the `universal_robot` package is installed. Since the package is not provided in the official ROS distributions, you can install it manually by cloning the repo:

```bash
cd /path/to/workspace/src
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
```

Then, build this package using the catkin build command:

```bash
catkin build force_torque_sensor
source /path/to/workspace/devel/setup.bash
```

Run the demo provided in this package with the following commands:

```bash
roslaunch force_torque_sensor demo_gazebo.launch
roslaunch force_torque_sensor demo_read_gazebo_sensor.launch
```

**Note: The demo is designed not to launch the Gazebo interface: the sensor broadcasts data even without the Gazebo interface.**

This demo has been tested with ROS Melodic and Gazebo 9.0.0.

## Gravity bias computation

All the sensors of the Automatic Control Group, except for the mock one, automatically cancel the gravity effect by computing a gravity bias. This is performed at the time the software object is created, so that all the mesures delivered by the sensors will be null until at least one of the two conditions below is verified:

- the orientation of the sensor changes
- the sensor gets in touch with the environment

If the sensor orientation changes after the object has been created, the user should call the function `computeBias` to re-compute the bias in order to retrieve null measures.

## Known issues

- When you launch `demo_gazebo.launch`, the following errors show up in the console, but they can be ignored:

```text
[ERROR] [1625124387.994541576, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_pan_joint
[ERROR] [1625124387.995757483, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_lift_joint
[ERROR] [1625124387.996846406, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/elbow_joint
[ERROR] [1625124387.997850029, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_1_joint
[ERROR] [1625124387.998926525, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_2_joint
[ERROR] [1625124387.999645983, 0.395000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_3_joint
```
