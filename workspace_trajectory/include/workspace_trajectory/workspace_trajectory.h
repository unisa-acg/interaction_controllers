/* -------------------------------------------------------------------
 *
 * This module has been developed as part of a collaboration between
 * the Automatic Control Group @ UNISA and ALTEC.
 *
 * Title:   workspace_trajectory.h 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Jul 20, 2018
 *
 * This class is twin to the WorkspaceTrajectory message and is aimed
 * at offering greater functionalities to generate and manipulate
 * trajectories prior to be transformed to messages to be exchanged
 * between nodes.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_WORKSPACE_TRAJECTORY_WORKSPACE_TRAJECTORY_H_
#define INCLUDE_WORKSPACE_TRAJECTORY_WORKSPACE_TRAJECTORY_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <workspace_trajectory_msgs/WorkspaceTrajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace workspace_trajectory
{

class WorkspaceTrajectory
{

public:

    /**
     * @brief Constructor that uses explicit trajectory parameters
     * 
     * All vectors must be of the same size and orientation is given with RPY convention.
     * 
     * @param name Trajectory name
     * @param time vector of timestamps
     * @param x vector of position x-coordinates
     * @param y vector of position y-coordinates
     * @param z vector of position z-coordinates
     * @param roll vector of roll angles
     * @param pitch vector of pitch angles
     * @param yaw vector of yaw angles
     */
    WorkspaceTrajectory(const std::string & name,
                        const std::vector<double> & time,
                        const std::vector<double> & x,
                        const std::vector<double> & y,
                        const std::vector<double> & z,
                        const std::vector<double> & roll,
                        const std::vector<double> & pitch,
                        const std::vector<double> & yaw);

    /**
     * @brief Constructor that builds a WorkspaceTrajectory object from a message
     * 
     * @param ws_trajectory_msg message from which to create the object
     */
    WorkspaceTrajectory(const workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg);

    /**
     * @brief Constructor that builds a WorkspaceTrajectory from waypoints and timestamps
     * 
     * @param name Trajectory name
     * @param waypoints vector of poses (position and orientation)
     * @param timestamps vector of timestamps for the poses in waypoints
     */
    WorkspaceTrajectory(const std::string & name,
                        const std::vector<geometry_msgs::Pose> & waypoints,
                        const std::vector<double> & timestamps);

    /**
     * @brief Constructs a WorkspaceTrajectory starting from a joint space trajectory, by using FW kinematics
     * 
     * @param name The name to be given to the generated trajectory
     * @param joint_space_trajectory The joint space trajectory from which to compute the task space trajectory
     * @param robot_state The robot state object to be used to call the FW kinematics function
     * @param planning_group The planning group whose EE corresponds to the task tip (for which to compute FW kinematics)
     */
    WorkspaceTrajectory(const std::string & name,
                        const moveit_msgs::RobotTrajectory & joint_space_trajectory,
                        const robot_state::RobotStatePtr robot_state,
                        const std::string & planning_group);

    /**
     * @brief Constructs a WorkspaceTrajectory loading it from file
     * 
     * @param name The name to be given to the generated trajectory
     * @param file_path Path and filename of the workspace trajectory in .traj format
     */
    WorkspaceTrajectory(const std::string & name,
                        const std::string & file_path);

    ~WorkspaceTrajectory() {};

    /**
     * @brief Get the list of waypoints
     * 
     * @return the list of waypoints as a vector of poses
     */
    const std::vector<geometry_msgs::Pose>& getWaypoints() const;

    /**
     * @brief Get the list of wrenches
     * 
     * @return the list of wrenches
     */
    const std::vector<geometry_msgs::Wrench>& getWrenches() const;

    /**
     * @brief Get the number of waypoints the trajectory is made of
     * 
     * @return Number of waypoints
     */
    int getNumberOfWaypoints() const;

    /**
     * @brief Convert trajectory to message
     * 
     * @param ws_trajectory_msg trajectory message to be initialized
     */
    void getWorkspaceTrajectoryMsg(workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg) const;

    /**
     * @brief Re-initialize trajectory from message
     * 
     * @param ws_trajectory_msg trajectory message to be converted to WorkspaceTrajectory object
     */
    void setWorkspaceTrajectoryMsg(const workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg);

    /**
     * @brief Perform a linear scaling of trajectory's timestamps
     * 
     * @param gradient Gradient of the linear scale
     */
    void scaleLinearly(float gradient);

    /**
     * @brief Reset trajectory duration with evenly spaced timestamps
     * 
     * First timestamp will be zero.
     * 
     * @param duration Trajectory duration to be set
     */
    void setEvenlySpacedTimestamps(double duration);

    /**
     * @brief Get trajectory duration
     * 
     * @return Trajectory duration (last minus first timestamp)
     */
    double getDuration() const;

    /**
     * @brief Get the curvilinear coordinate of the path for each waypoint
     * 
     * @return Curvilinear coordinate values for each waypoint
     */
    std::vector<double> getCurvilinearCoordinate() const;

    /**
     * @brief Compute the tangent vector for a given waypoint
     * 
     * The return type is Eigen::VectorXd because ROS/tf2 does not support
     * representing Cartesian vectors with a specific representation for
     * the orientation (RPY in this case).
     * 
     * @param index Waypoint index at which to compute the tangent vector
     * @return Six-dimensional tangent vector at given waypoint whose orientation coordinates are represented with roll-pitch-yaw angles
     */
    Eigen::VectorXd getTangentVectorRPYAtWaypoint(unsigned int index) const;

    /**
     * @brief Compute the tangent vector for a given waypoint
     * 
     * Eigen::VectorXd is preferred to ROS's Twist because the tangent
     * vector is not exactly a twist.
     * 
     * @param index Waypoint index at which to compute the tangent vector
     * @return Six-dimensional tangent vector at given waypoint represented as pseudo-twist
     */
    Eigen::VectorXd getTangentVectorAtWaypoint(unsigned int index) const;

    /**
     * @brief Exports trajectory to .traj file
     * 
     * @param file_path Full path of the output file
     */
    void exportToBinary(const std::string & file_path) const;

    /**
     * @brief Prints timestamps to stdout
     */
    void printTimestamps() const;

private:

    /**
     * @brief Verifies that timestamps are an increasing monotone function
     */
    void checkTimestamps_() const;

    std::string name_;
    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<geometry_msgs::Wrench> wrenches_;
    std::vector<double> timestamps_;
    unsigned int length_;
};

}

#endif /* INCLUDE_WORKSPACE_TRAJECTORY_WORKSPACE_TRAJECTORY_H_ */
