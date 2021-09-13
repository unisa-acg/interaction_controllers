/* -------------------------------------------------------------------
 *
 * This module has been developed as part of a collaboration between
 * the Automatic Control Group @ UNISA and ALTEC.
 *
 * Title:   workspace_trajectory.cpp 
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Jul 20, 2018
 *
 * See workspace_trajectory.h for a description of the class.
 *
 * -------------------------------------------------------------------
 */

#include <workspace_trajectory/workspace_trajectory.h>
#include <workspace_trajectory/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <angles/angles.h>
#include <boost/filesystem.hpp>
#include <fstream>

// Data types conversions
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>    // Conversions between tf2 and geometry_msgs
#include <tf2_eigen/tf2_eigen.h>                    // Conversions between tf2 and Eigen
#include <tf2_kdl/tf2_kdl.h>                        // Conversions between tf2 and KDL

using namespace workspace_trajectory;

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const std::vector<double> & time,
    const std::vector<double> & x,
    const std::vector<double> & y,
    const std::vector<double> & z,
    const std::vector<double> & roll,
    const std::vector<double> & pitch,
    const std::vector<double> & yaw):
            name_(name),
            timestamps_(time),
            length_(time.size()),
            wrenches_(length_)
{
    if( x.size() != length_ || y.size() != length_ || z.size() != length_ || roll.size() != length_ || pitch.size() != length_ || yaw.size() != length_ )
        throw workspace_trajectory::Exception("Trajectories for single coordinates must be of the same size");

    checkTimestamps_();

    for(int i=0; i<length_; i++)
    {
        tf2::Quaternion q;
        q.setRPY(roll[i], pitch[i], yaw[i]);

        geometry_msgs::Pose pose;
        pose.position.x = x[i];
        pose.position.y = y[i];
        pose.position.z = z[i];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        waypoints_.push_back(pose);
    }
}

WorkspaceTrajectory::WorkspaceTrajectory(const workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg)
{
    setWorkspaceTrajectoryMsg(ws_trajectory_msg);
}

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const std::vector<geometry_msgs::Pose> & waypoints,
    const std::vector<double> & timestamps):
        name_(name),
        waypoints_(waypoints),
        timestamps_(timestamps),
        length_(timestamps.size()),
        wrenches_(length_)
{
    if(waypoints.size() != length_)
        throw workspace_trajectory::Exception("The time vector must have as many elements as the waypoints");

    checkTimestamps_();
}

WorkspaceTrajectory::WorkspaceTrajectory(
    const std::string & name,
    const moveit_msgs::RobotTrajectory & jst,
    const robot_state::RobotStatePtr rs,
    const std::string & planning_group):
        name_(name),
        length_(jst.joint_trajectory.points.size()),
        wrenches_(length_)
{
    ROS_INFO_NAMED("WorkspaceTrajectory", "Creating trajectory with %u waypoints", length_);

    const robot_state::JointModelGroup* jmg = rs->getJointModelGroup(planning_group);

    for(int i=0; i<length_; i++)
    {
        trajectory_msgs::JointTrajectoryPoint point = jst.joint_trajectory.points[i];

        rs->setJointGroupPositions(jmg, point.positions);

        const Eigen::Affine3d & ee_pose = rs->getGlobalLinkTransform(jmg->getLinkModelNames().back());

        geometry_msgs::Pose pose_msg = tf2::toMsg(ee_pose);

        waypoints_.push_back(pose_msg);

        timestamps_.push_back(point.time_from_start.toSec());
    }

    checkTimestamps_();

    ROS_INFO_NAMED("WorkspaceTrajectory", "Created trajectory '%s' with %u waypoints", name_.c_str(), length_);
}

WorkspaceTrajectory::WorkspaceTrajectory(
        const std::string & name,
        const std::string & file_path):
                name_(name)
{
    if(file_path == "" || !boost::filesystem::exists(file_path))
        throw workspace_trajectory::Exception("File path is not set or the file does not exist");

    std::ifstream in_file(file_path, std::ifstream::in | std::ifstream::binary);

    in_file.read(reinterpret_cast<char *>(&length_), sizeof(uint32_t));

    for(int i=0; i < length_; i++)
    {
        double timestamp;
        geometry_msgs::Pose pose;

        in_file.read(reinterpret_cast<char *>(&timestamp), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.position.x), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.position.y), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.position.z), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.orientation.x), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.orientation.y), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.orientation.z), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&pose.orientation.w), sizeof(double));

        timestamps_.push_back(timestamp);
        waypoints_.push_back(pose);

        geometry_msgs::Wrench wrench;

        in_file.read(reinterpret_cast<char *>(&wrench.force.x), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&wrench.force.y), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&wrench.force.z), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&wrench.torque.x), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&wrench.torque.y), sizeof(double));
        in_file.read(reinterpret_cast<char *>(&wrench.torque.z), sizeof(double));

        wrenches_.push_back(wrench);
    }

    ROS_INFO("Loaded trajectory of %d waypoints", length_);

    in_file.close();

    checkTimestamps_();
}

const std::vector<geometry_msgs::Pose>& WorkspaceTrajectory::getWaypoints() const
{
    return waypoints_;
}

const std::vector<geometry_msgs::Wrench>& WorkspaceTrajectory::getWrenches() const
{
    return wrenches_;
}

int WorkspaceTrajectory::getNumberOfWaypoints() const
{
    return length_;
}

void WorkspaceTrajectory::getWorkspaceTrajectoryMsg(workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg) const
{
    ws_trajectory_msg.name = name_;
    ws_trajectory_msg.waypoints = waypoints_;
    ws_trajectory_msg.timestamps = timestamps_;
    ws_trajectory_msg.wrenches = wrenches_;
}

void WorkspaceTrajectory::setWorkspaceTrajectoryMsg(const workspace_trajectory_msgs::WorkspaceTrajectory & ws_trajectory_msg)
{
    if(ws_trajectory_msg.waypoints.size() != ws_trajectory_msg.timestamps.size())
        throw workspace_trajectory::Exception("The message must have as many timestamps as waypoints");

    if(ws_trajectory_msg.wrenches.size() != 0 && ws_trajectory_msg.wrenches.size() != ws_trajectory_msg.timestamps.size())
        throw workspace_trajectory::Exception("The message must have as many timestamps as wrenches, otherwise set wrench vector size to zero if wrenches are not used.");

    name_ = ws_trajectory_msg.name;
    waypoints_ = ws_trajectory_msg.waypoints;
    wrenches_ = ws_trajectory_msg.wrenches;
    timestamps_ = ws_trajectory_msg.timestamps;
    length_ = ws_trajectory_msg.waypoints.size();

    if(wrenches_.size() == 0)
        wrenches_.resize(length_);

    checkTimestamps_();
}

void WorkspaceTrajectory::scaleLinearly(float gradient)
{
    for(int i=0; i<length_; i++)
    {
        timestamps_[i] = timestamps_[i] * gradient;
    }

    checkTimestamps_();
}

void WorkspaceTrajectory::setEvenlySpacedTimestamps(double duration)
{
    if(duration <= 0.0)
        throw workspace_trajectory::Exception("Trajectory duration must be greater than zero");

    std::vector<double> time_range(length_);

    double step = duration/(length_-1);
    int i = 0;

    while(timestamps_[0] <= timestamps_[0]+duration)
    {
        time_range[i++] = timestamps_[0];
        timestamps_[0] += step;
    }

    if(i < length_)
    {
        time_range[i] = timestamps_[0]+duration;
    }

    timestamps_ = time_range;
}

double WorkspaceTrajectory::getDuration() const
{
    if(length_ == 0)
        return 0;

    return (timestamps_[length_-1] - timestamps_[0]);
}

std::vector<double> WorkspaceTrajectory::getCurvilinearCoordinate() const
{
    std::vector<double> curvilinear_coordinate(length_);
    curvilinear_coordinate[0] = 0;

    tf2::Transform prev_pose, curr_pose;
    
    tf2::fromMsg(waypoints_[0], prev_pose);

    for(unsigned int i = 1; i < length_; i++)
    {
        tf2::fromMsg(waypoints_[i], curr_pose);
        double distance = prev_pose.getOrigin().distance(curr_pose.getOrigin());
        curvilinear_coordinate[i] = curvilinear_coordinate[i-1] + distance;
        prev_pose = curr_pose;
    }

    return curvilinear_coordinate;
}

Eigen::VectorXd WorkspaceTrajectory::getTangentVectorRPYAtWaypoint(unsigned int index) const
{
    if(index >= length_)
        throw workspace_trajectory::Exception("Index is not valid");

    if(index == length_ - 1)
        throw workspace_trajectory::Exception("Tangent vector is undetermined for last waypoint");

    Eigen::Vector3d position_curr, position_next;
    tf2::Quaternion orientation_curr, orientation_next;
    Eigen::VectorXd tangent_vector(6);

    tf2::fromMsg(waypoints_[index].position, position_curr);
    tf2::fromMsg(waypoints_[index].orientation, orientation_curr);
    tf2::fromMsg(waypoints_[index+1].position, position_next);
    tf2::fromMsg(waypoints_[index+1].orientation, orientation_next);

    tf2::Matrix3x3 m_orientation_curr(orientation_curr);
    tf2::Matrix3x3 m_orientation_next(orientation_next);
    
    tf2Scalar roll_curr, pitch_curr, yaw_curr;
    tf2Scalar roll_next, pitch_next, yaw_next;

    m_orientation_curr.getRPY(roll_curr, pitch_curr, yaw_curr);
    m_orientation_next.getRPY(roll_next, pitch_next, yaw_next);

    tangent_vector(0) = position_next(0) - position_curr(0);
    tangent_vector(1) = position_next(1) - position_curr(1);
    tangent_vector(2) = position_next(2) - position_curr(2);

    tangent_vector(3) = angles::shortest_angular_distance(roll_curr, roll_next);
    tangent_vector(4) = angles::shortest_angular_distance(pitch_curr, pitch_next);
    tangent_vector(5) = angles::shortest_angular_distance(yaw_curr, yaw_next);
 
    return tangent_vector/tangent_vector.norm();
}

Eigen::VectorXd WorkspaceTrajectory::getTangentVectorAtWaypoint(unsigned int index) const
{
    if(index >= length_)
        throw workspace_trajectory::Exception("Index is not valid");

    if(index == length_ - 1)
        throw workspace_trajectory::Exception("Tangent vector is undetermined for last waypoint");

    KDL::Frame current_frame, next_frame;

    tf2::fromMsg(waypoints_[index], current_frame);
    tf2::fromMsg(waypoints_[index+1], next_frame);

    KDL::Twist pseudo_twist = KDL::diff(current_frame, next_frame);

    Eigen::VectorXd tangent_vector(6);

    tangent_vector(0) = pseudo_twist.vel[0];
    tangent_vector(1) = pseudo_twist.vel[1];
    tangent_vector(2) = pseudo_twist.vel[2];
    tangent_vector(3) = pseudo_twist.rot[0];
    tangent_vector(4) = pseudo_twist.rot[1];
    tangent_vector(5) = pseudo_twist.rot[2];

    return tangent_vector/tangent_vector.norm();
}

void WorkspaceTrajectory::exportToBinary(const std::string & file_path) const
{
    std::ofstream out_file(file_path, std::ofstream::out | std::ofstream::binary);

    if(out_file.rdstate() != std::ios_base::goodbit)
        throw workspace_trajectory::Exception("Could not create the output file");

    out_file.write(reinterpret_cast<const char *>(&length_), sizeof(unsigned int));

    for(int i=0; i < length_; i++)
    {
        out_file.write(reinterpret_cast<const char *>(&timestamps_[i]), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].position.x), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].position.y), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].position.z), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].orientation.x), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].orientation.y), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].orientation.z), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&waypoints_[i].orientation.w), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].force.x), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].force.y), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].force.z), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].torque.x), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].torque.y), sizeof(double));
        out_file.write(reinterpret_cast<const char *>(&wrenches_[i].torque.z), sizeof(double));
    }

    out_file.close();
}

void WorkspaceTrajectory::printTimestamps() const
{
    for(int i=0; i<length_; i++)
        ROS_INFO("Index = %d, Time = %f", i, timestamps_[i]);
}

void WorkspaceTrajectory::checkTimestamps_() const
{
    if(timestamps_.size() == 0)
        throw workspace_trajectory::Exception("No timestamps available");

    if(timestamps_.size() > 1)
    {
        for(int i=1; i<timestamps_.size(); i++)
        {
            if(timestamps_[i] <= timestamps_[i-1])
                throw workspace_trajectory::Exception("Time is not an increasing monotone function");
        }
    }
}
