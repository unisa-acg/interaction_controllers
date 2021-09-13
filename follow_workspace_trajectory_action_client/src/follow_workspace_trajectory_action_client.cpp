/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   follow_workspace_trajectory_action_client.cpp
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    Apr 26, 2020
 *
 * This node instantiates an action client in order to communicate
 * with an action server using the FollowWorkspaceTrajectory action.
 * This way, the trajectory execution can be monitored and
 * data logged to bagfile. The node opens a file in the '.traj' format
 * with a workspace trajectory and sends it to the controller.
 * A bagfile is written containing the desired, actual and error
 * trajectories.
 *
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

#include <actionlib/client/simple_action_client.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include <acg_control_msg/FollowWorkspaceTrajectoryAction.h>
#include <workspace_trajectory/workspace_trajectory.h>

std::string nowStr();

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const acg_control_msg::FollowWorkspaceTrajectoryResultConstPtr & result);
void handleGoalActiveEvent();
void handleFeedbackEvent(const acg_control_msg::FollowWorkspaceTrajectoryFeedbackConstPtr & feedback);

workspace_trajectory_msgs::WorkspaceTrajectory desired_trajectory;
workspace_trajectory_msgs::WorkspaceTrajectory actual_trajectory;
workspace_trajectory_msgs::WorkspaceTrajectory error_trajectory;

bool success;
int number_feedback_messages_received;
int fraction_feedback_messages_to_save;

int main(int argc, char **argv)
{
    // Initializing the node
    ros::init(argc, argv, "follow_workspace_trajectory_action_client");
    ros::NodeHandle node_handle;

    std::shared_ptr<workspace_trajectory::WorkspaceTrajectory> workspace_trajectory = nullptr;

    std::string output_path = ros::package::getPath("follow_workspace_trajectory_action_client") + "/output/";

    if(!boost::filesystem::exists(output_path))
        boost::filesystem::create_directory(output_path);

    // Get the action topic from the parameter server
    std::string action_topic;
    if(!node_handle.getParam("/action_topic", action_topic))
        ROS_ERROR("Could not find the action topic on the parameter server");

    // Get the path for input trajectory
    std::string trajectory_file_path;
    ros::param::get("~filename", trajectory_file_path);
    trajectory_file_path = ros::package::getPath("follow_workspace_trajectory_action_client") + "/trajectories/" + trajectory_file_path;

    // Get the fraction of feedback messages to save
    if(!node_handle.getParam("/fraction_feedback_messages_to_save", fraction_feedback_messages_to_save))
        ROS_ERROR("Could not find parameter 'fraction_feedback_messages_to_save'");

    if(fraction_feedback_messages_to_save <= 0 || fraction_feedback_messages_to_save > 100)
    {
        ROS_ERROR("'fraction_feedback_messages_to_save' should be greater than zero and not greater than 100");
        ros::shutdown();
        exit(1);
    }

    try
    {
        workspace_trajectory = std::make_shared<workspace_trajectory::WorkspaceTrajectory>("workspace_trajectory", trajectory_file_path);
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
        ros::shutdown();
        exit(1);
    }

    // Create the action client and wait for the action server
    actionlib::SimpleActionClient<acg_control_msg::FollowWorkspaceTrajectoryAction> client(action_topic, true);

    if(!client.waitForServer(ros::Duration(30.0)))
        ROS_ERROR("Could not connect to the server. Timeout elapsed.");

    acg_control_msg::FollowWorkspaceTrajectoryGoal trajectory_goal;
    workspace_trajectory->getWorkspaceTrajectoryMsg(trajectory_goal.workspace_trajectory);

    // Extract setpoint from trajectory
    acg_control_msg::FollowWorkspaceTrajectoryGoal set_point_goal;

    set_point_goal.workspace_trajectory.waypoints.resize(1);
    set_point_goal.workspace_trajectory.waypoints[0] = trajectory_goal.workspace_trajectory.waypoints[0];

    set_point_goal.workspace_trajectory.wrenches.resize(1);
    set_point_goal.workspace_trajectory.wrenches[0] = trajectory_goal.workspace_trajectory.wrenches[0];

    set_point_goal.workspace_trajectory.timestamps.resize(1);
    set_point_goal.workspace_trajectory.timestamps[0] = 0;

    // Sending the goal to the action server and waiting for the trajectory to complete
    client.sendGoal(set_point_goal, &handleGoalCompletionEvent);

    success = false;
    if(!client.waitForResult(ros::Duration(30.0)) || !success)
    {
        ROS_ERROR("The robot was not able to reach the set point");
        ros::shutdown();
        exit(1);
    }
    
    ROS_INFO("Set point reached");

    number_feedback_messages_received = 0;

    // Waiting some time before to be sure that the robot is stable before executing the whole trajectory 
    ros::Duration(3).sleep();

    client.sendGoal(trajectory_goal, &handleGoalCompletionEvent, &handleGoalActiveEvent, &handleFeedbackEvent);

    success = false;
    if(!client.waitForResult(ros::Duration(200.0)) || !success)
        ROS_ERROR("The robot did not achieve the goal in the allotted time");
    else
        ROS_INFO("Trajectory completed!");

    // Writing trajectory logs to bagfiles
    std::string output_filename = output_path +  "follow_workspace_trajectory_result_" + nowStr() + ".bag";

    rosbag::Bag output_bag(output_filename, rosbag::bagmode::Write);
    output_bag.write("follow_workspace_trajectory_result", ros::Time::now(), desired_trajectory);
    output_bag.write("follow_workspace_trajectory_result", ros::Time::now(), actual_trajectory);
    output_bag.write("follow_workspace_trajectory_result", ros::Time::now(), error_trajectory);
    output_bag.close();

    ROS_INFO_STREAM("Generated bagfile " << output_filename.c_str());

    ros::shutdown();
    exit(0);
}

std::string nowStr()
{
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

void handleGoalCompletionEvent(const actionlib::SimpleClientGoalState & state, const acg_control_msg::FollowWorkspaceTrajectoryResultConstPtr & result)
{
    success = result->success;
}

void handleGoalActiveEvent()
{
    ROS_INFO("Trajectory received by the controller!");
}

void handleFeedbackEvent(const acg_control_msg::FollowWorkspaceTrajectoryFeedbackConstPtr & feedback)
{
    number_feedback_messages_received++;

    if(number_feedback_messages_received % fraction_feedback_messages_to_save == 0)
    {
        ros::Duration time_from_start(feedback->time_from_start.data.sec, feedback->time_from_start.data.nsec);
        double stamp = time_from_start.toSec();

        actual_trajectory.waypoints.push_back(feedback->actual_pose);
        actual_trajectory.wrenches.push_back(feedback->actual_wrench);
        actual_trajectory.timestamps.push_back(stamp);

        desired_trajectory.waypoints.push_back(feedback->desired_pose);
        desired_trajectory.wrenches.push_back(feedback->desired_wrench);
        desired_trajectory.timestamps.push_back(stamp);

        error_trajectory.waypoints.push_back(feedback->error_pose);
        error_trajectory.wrenches.push_back(feedback->error_wrench);
        error_trajectory.timestamps.push_back(stamp);
    }
}