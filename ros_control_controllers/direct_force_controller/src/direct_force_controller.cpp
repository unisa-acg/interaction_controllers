/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   direct_force_controller.cpp
 * Author:  Sabatino Simeone, Giovanni Longobardi
 * Org.:    UNISA
 * Date:    Dec 20, 2019
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <angles/angles.h>
#include <direct_force_controller/direct_force_controller.h>
#include <direct_force_controller/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace direct_force_controller
{

DirectForceController::DirectForceController():
    robot_model_loader_(nullptr),
    kinematic_model_(nullptr),
    kinematic_state_(nullptr),
    action_server_(nullptr),
    trajectory_received_(false),
    number_reached_trajectory_points_(0)
{
    ros::NodeHandle nh;

    force_torque_sensor_loader_ = std::make_unique<pluginlib::ClassLoader<force_torque_sensor::ForceTorqueSensor>>("force_torque_sensor", "force_torque_sensor::ForceTorqueSensor");

    std::string force_torque_sensor;
    if (!nh.getParam("/force_torque_sensor_type", force_torque_sensor))
        throw direct_force_controller::Exception("Cannot find sensor type on the parameter server");

    try
    {
        force_torque_sensor_ = force_torque_sensor_loader_->createInstance("force_torque_sensor::" + force_torque_sensor);
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
    }
}

DirectForceController::~DirectForceController()
{
}

bool DirectForceController::init(hardware_interface::PositionJointInterface* robot_hw, ros::NodeHandle& controller_nh)
{
    joint_position_handles_.clear();
    action_server_.reset(new actionlib::SimpleActionServer<acg_control_msg::FollowWorkspaceTrajectoryAction>(controller_nh, "/direct_force_controller", false));

    std::string ns = controller_nh.getNamespace();

    double loop_frequency;
    if (!controller_nh.getParam(ns + "/loop_frequency",loop_frequency))
        throw direct_force_controller::Exception("Cannot read loop frequency from parameter server");

    // Set the queue size at the double of the frequency not to lose feedback messages
    controller_nh.setParam(ns + "/direct_force_controller/actionlib_client_sub_queue_size", 2*loop_frequency);

    action_server_->registerGoalCallback(boost::bind(&DirectForceController::getActionGoal_, this));

    robot_model_loader_ = std::make_shared<const robot_model_loader::RobotModelLoader>("robot_description");
    kinematic_model_ = robot_model_loader_->getModel();

    kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);

    std::string joint_model_group_name;
    if (!controller_nh.getParam("/robot_parameters/joint_model_group", joint_model_group_name))
        throw direct_force_controller::Exception("Cannot read joint model group name from parameter server");

    joint_model_group_ = kinematic_model_->getJointModelGroup(joint_model_group_name.c_str());

    const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
    number_of_joints_ = joint_names.size();

    for (int i = 0; i < number_of_joints_; i++)
        joint_position_handles_.push_back(robot_hw->getHandle(joint_names[i]));

    actual_joint_positions_.resize(number_of_joints_);

    if (!controller_nh.getParam("/robot_parameters/base_frame_name", base_frame_name_))
        throw direct_force_controller::Exception("Cannot read base frame name from parameter server");

    world_to_base_transform_ = kinematic_state_->getFrameTransform("/" + base_frame_name_);

    Eigen::Isometry3d world_to_end_effector_transform = kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());

    std::string force_torque_sensor_mounting_frame;
    if (!controller_nh.getParam("/robot_parameters/force_torque_sensor_mounting_frame", force_torque_sensor_mounting_frame))
        throw direct_force_controller::Exception("Cannot read force/torque sensor frame name from parameter server");
    
    std::vector<double> ft_sensor_frame_orientation(3);
    if (!controller_nh.getParam("/robot_parameters/force_torque_sensor_frame_orientation", ft_sensor_frame_orientation))
        throw direct_force_controller::Exception("Cannot read the orientation of the force/torque sensor frame from parameter server");

    // Compute the quaternion from RPY angles, corresponding to a rotation 
    // around X (roll) then around Y (pitch) and then around Z (yaw) in fixed frame
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(ft_sensor_frame_orientation[0], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ft_sensor_frame_orientation[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ft_sensor_frame_orientation[2], Eigen::Vector3d::UnitX());
    Eigen::Matrix3d force_torque_sensor_rotation = q.toRotationMatrix();

    end_effector_to_ft_sensor_transform_ = world_to_end_effector_transform.inverse() * kinematic_state_->getFrameTransform("/" + force_torque_sensor_mounting_frame) * force_torque_sensor_rotation;

    if (!controller_nh.getParam(ns + "/sp_tolerance", sp_tolerance_))
        throw direct_force_controller::Exception("Cannot read set point position tolerance from parameter server");

    if (!controller_nh.getParam(ns + "/ik_timeout", ik_timeout_))
        throw direct_force_controller::Exception("Cannot read IK timeout from parameter server");

    if (!controller_nh.getParam(ns + "/ik_tolerance", ik_tolerance_))
        throw direct_force_controller::Exception("Cannot read IK tolerance from parameter server");

    if (!controller_nh.getParam(ns + "/force_control_along_x", force_control_along_x_))
        throw direct_force_controller::Exception("Cannot read 'force control along x' from parameter server");

    if (!controller_nh.getParam(ns + "/force_control_along_y", force_control_along_y_))
        throw direct_force_controller::Exception("Cannot read 'force control along y' from parameter server");

    if (!controller_nh.getParam(ns + "/force_control_along_z", force_control_along_z_))
        throw direct_force_controller::Exception("Cannot read 'force control along z' from parameter server");

    double pid_p;
    if (!controller_nh.getParam(ns + "/pid_p", pid_p))
        throw direct_force_controller::Exception("Cannot read proportional gain from parameter server");

    double pid_i;
    if (!controller_nh.getParam(ns + "/pid_i", pid_i))
        throw direct_force_controller::Exception("Cannot read integral gain from parameter server");

    double pid_d;
    if (!controller_nh.getParam(ns + "/pid_d", pid_d))
        throw direct_force_controller::Exception("Cannot read derivative gain from parameter server");

    pid_x_.initPid(pid_p, pid_i, pid_d, 1.0, -1.0);
    pid_y_.initPid(pid_p, pid_i, pid_d, 1.0, -1.0);
    pid_z_.initPid(pid_p, pid_i, pid_d, 1.0, -1.0);

    joint_positions_command_.resize(number_of_joints_);

    action_server_->start();

    return true;
}

void DirectForceController::starting(const ros::Time& time)
{
    f_d_ = Eigen::Vector3d::Zero();
    x_d_e_ =  Eigen::Vector3d::Zero();

    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();

    for (int i = 0; i < number_of_joints_; i++)
        actual_joint_positions_[i] = joint_position_handles_[i].getPosition();

    kinematic_state_->setJointGroupPositions(joint_model_group_, actual_joint_positions_);

    // Command current position to keep the robot at rest when starting
    x_c_b_ = world_to_base_transform_.inverse() * kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());

    base_to_end_effector_transform_ = x_c_b_;
    x_d_b_ = x_c_b_.translation();

    active_state_ = State::static_direct_force_control;
}

void DirectForceController::update(const ros::Time& time, const ros::Duration& period)
{
    // Reading phase
    for (int i = 0; i < number_of_joints_; i++)
        actual_joint_positions_[i] = joint_position_handles_[i].getPosition();

    kinematic_state_->setJointGroupPositions(joint_model_group_, actual_joint_positions_);

    // Update
    switch(active_state_)
    {
        case State::independent_joint_control:
        {
            if(isPositionReached_(actual_joint_positions_))
            {
                number_reached_trajectory_points_ = 0;
                x_d_e_ =  Eigen::Vector3d::Zero();
                
                base_to_end_effector_transform_ = world_to_base_transform_.inverse() * kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());
                x_d_b_ = base_to_end_effector_transform_.translation();

                force_torque_sensor_->computeBias();

                result_.success  = true;
                action_server_->setSucceeded(result_);

                active_state_ = State::static_direct_force_control;
            }
        }
        break;

        case State::trajectory_execution_direct_force_control:
        {
            updateActionFeedback_();
            number_reached_trajectory_points_++;
            if (number_reached_trajectory_points_ == reference_ws_trajectory_.waypoints.size())
            {
                number_reached_trajectory_points_ = 0;

                result_.success  = true;
                action_server_->setSucceeded(result_);

                active_state_ = State::static_direct_force_control;
            }
            else
            {
                x_d_b_[x] = reference_ws_trajectory_.waypoints[number_reached_trajectory_points_].position.x;
                x_d_b_[y] = reference_ws_trajectory_.waypoints[number_reached_trajectory_points_].position.y;
                x_d_b_[z] = reference_ws_trajectory_.waypoints[number_reached_trajectory_points_].position.z;

                x_d_e_ = base_to_end_effector_transform_.inverse() * x_d_b_;

                f_d_(x) = reference_ws_trajectory_.wrenches[number_reached_trajectory_points_].force.x;
                f_d_(y) = reference_ws_trajectory_.wrenches[number_reached_trajectory_points_].force.y;
                f_d_(z) = reference_ws_trajectory_.wrenches[number_reached_trajectory_points_].force.z;
            }
        }

        // This can be an independent state or a sub-state of trajectory_execution_direct_force_control
        case State::static_direct_force_control:
        {
            // Event received
            if(trajectory_received_)
            {
                if(reference_ws_trajectory_.waypoints.size() == 1)
                {
                    x_c_b_.translation()(x) = reference_ws_trajectory_.waypoints[0].position.x;
                    x_c_b_.translation()(y) = reference_ws_trajectory_.waypoints[0].position.y;
                    x_c_b_.translation()(z) = reference_ws_trajectory_.waypoints[0].position.z;
                    x_c_b_.matrix().topLeftCorner<3,3>() = Eigen::Matrix3d(Eigen::Quaterniond(reference_ws_trajectory_.waypoints[0].orientation.w, 
                                                                                              reference_ws_trajectory_.waypoints[0].orientation.x, 
                                                                                              reference_ws_trajectory_.waypoints[0].orientation.y, 
                                                                                              reference_ws_trajectory_.waypoints[0].orientation.z));

                    f_d_(x) = reference_ws_trajectory_.wrenches[0].force.x;
                    f_d_(y) = reference_ws_trajectory_.wrenches[0].force.y;
                    f_d_(z) = reference_ws_trajectory_.wrenches[0].force.z;

                    active_state_ = State::independent_joint_control;
                }
                else
                {
                    active_state_ = State::trajectory_execution_direct_force_control;
                }

                pid_x_.reset();
                pid_y_.reset();
                pid_z_.reset();
                trajectory_received_ = false;
            }
            else
            {
                std::vector<float> force_torque_sensor_measure(6, 0.0);

                force_torque_sensor_->getAverageMeasure(force_torque_sensor_measure);

                // The force excerted by the robot is equal and opposite to the measure
                f_e_(x) = -force_torque_sensor_measure[x];
                f_e_(y) = -force_torque_sensor_measure[y];
                f_e_(z) = -force_torque_sensor_measure[z];

                delta_f_ = f_d_ - end_effector_to_ft_sensor_transform_.rotation() * f_e_;

                // Force control
                x_c_(x) = force_control_along_x_ ? pid_x_.computeCommand(delta_f_(x), period) : x_d_e_(x);
                x_c_(y) = force_control_along_y_ ? pid_y_.computeCommand(delta_f_(y), period) : x_d_e_(y);
                x_c_(z) = force_control_along_z_ ? pid_z_.computeCommand(delta_f_(z), period) : x_d_e_(z);

                x_c_b_.translation() = base_to_end_effector_transform_.translation() + base_to_end_effector_transform_.rotation() * x_c_;
            }
        }
    }

    if (!computeInverseKinematics_(x_c_b_, joint_positions_command_))
    {
        if(active_state_ != State::static_direct_force_control)
        {
            result_.success = false;
            action_server_->setAborted(result_);
        }
        throw direct_force_controller::Exception("Did not find a valid IK solution");
    }

    for (int i = 0; i < number_of_joints_; i++)
        joint_position_handles_[i].setCommand(joint_positions_command_[i]);
}

void DirectForceController::stopping(const ros::Time&)
{
}

bool DirectForceController::isPositionReached_(const std::vector<double>& joint_values) const
{
    std::vector<double> joint_differences(number_of_joints_);
    computeVectorsDifference_(joint_differences, joint_positions_command_, joint_values, joint_model_group_);

    for (int i = 0; i < number_of_joints_; i++)
        if(fabs(joint_differences[i]) > sp_tolerance_)
            return false;

    return true;
}

bool DirectForceController::computeInverseKinematics_(Eigen::Isometry3d pose, std::vector<double>& joint_values)
{
    // Since the reference frame of the pose may be different from the frame used for the kinematic inversion,
    // the pose is set to be expressed with respect to the IK solver frame.
    kinematic_state_->setToIKSolverFrame(pose, "/" + base_frame_name_);
    
    if(!kinematic_state_->setFromIK(joint_model_group_, pose, ik_timeout_))
        return false;
    
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);

    // For safety reasons, check if the IK solution is close to the previous solution
    std::vector<double> joint_differences(number_of_joints_);
    computeVectorsDifference_(joint_differences, joint_values, actual_joint_positions_, joint_model_group_);

    for (int i = 0; i < number_of_joints_; i++)
        if(fabs(joint_differences[i]) > ik_tolerance_)
            return false;

    return true;
}

void DirectForceController::computeVectorsDifference_(
    std::vector<double>& diff,
    const std::vector<double>& minuend,
    const std::vector<double>& subtrahend,
    const moveit::core::JointModelGroup* joint_model_group) const
{
    for (int i = 0; i < minuend.size(); i++)
    {
        if (joint_model_group->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            // Compute difference between revolute joints
            robot_model::VariableBounds bounds = joint_model_group->getParentModel().getVariableBounds(joint_model_group->getVariableNames()[i]);

            if(bounds.position_bounded_)
            {
                angles::shortest_angular_distance_with_large_limits(subtrahend[i], minuend[i], bounds.min_position_, bounds.max_position_, diff[i]);
            }
            else
                diff[i] = angles::shortest_angular_distance(subtrahend[i], minuend[i]);
        }
        else if (joint_model_group->getActiveJointModels()[i]->getType() == robot_model::JointModel::PLANAR)
        {
            throw direct_force_controller::Exception("Planar joints are not currently supported.");
        }
        else
        {
            // ! Other joint model types are included here (PRISMATIC, FIXED, UNKNOWN, etc.)
            diff[i] = minuend[i] - subtrahend[i];
        }
    }
}

void DirectForceController::getActionGoal_()
{
    if(active_state_ == State::static_direct_force_control)
    {
        // Trajectory received from the action client
        reference_ws_trajectory_ = action_server_->acceptNewGoal()->workspace_trajectory;

        trajectory_received_ = true;
    }
}

void DirectForceController::updateActionFeedback_()
{
    acg_control_msg::FollowWorkspaceTrajectoryFeedback feedback;

    ros::Duration time_from_start;

    if(number_reached_trajectory_points_ == 0)
    {
        time_from_start = ros::Duration(0);
        first_timestamp_ = ros::Time::now();
    }
    else
    {
        time_from_start = ros::Time::now() - first_timestamp_;
    }

    Eigen::Isometry3d end_effector_state = world_to_base_transform_.inverse() * kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());

    // Stamp the feedback message
    feedback.time_from_start.data.sec = time_from_start.sec;
    feedback.time_from_start.data.nsec = time_from_start.nsec;

    // Update desired trajectory feedback
    feedback.desired_pose = reference_ws_trajectory_.waypoints[number_reached_trajectory_points_];
    feedback.desired_wrench = reference_ws_trajectory_.wrenches[number_reached_trajectory_points_];

    // Update actual trajectory feedback
    feedback.actual_pose = tf2::toMsg(end_effector_state);

    feedback.actual_wrench.force.x = (end_effector_to_ft_sensor_transform_.rotation() * f_e_)(x);
    feedback.actual_wrench.force.y = (end_effector_to_ft_sensor_transform_.rotation() * f_e_)(y);
    feedback.actual_wrench.force.z = (end_effector_to_ft_sensor_transform_.rotation() * f_e_)(z);
    feedback.actual_wrench.torque.x = 0;
    feedback.actual_wrench.torque.y = 0;
    feedback.actual_wrench.torque.z = 0;

    // Update error trajectory feedback
    tf2::Transform desired_transform, actual_transform, error_transform;
    tf2::fromMsg(feedback.desired_pose, desired_transform);
    tf2::fromMsg(feedback.actual_pose, actual_transform);
    error_transform = actual_transform.inverseTimes(desired_transform);
    tf2::toMsg(error_transform, feedback.error_pose);

    feedback.error_wrench.force.x = feedback.desired_wrench.force.x - feedback.actual_wrench.force.x;
    feedback.error_wrench.force.y = feedback.desired_wrench.force.y - feedback.actual_wrench.force.y;
    feedback.error_wrench.force.z = feedback.desired_wrench.force.z - feedback.actual_wrench.force.z;
    feedback.error_wrench.torque.x = feedback.desired_wrench.torque.x - feedback.actual_wrench.torque.x;
    feedback.error_wrench.torque.y = feedback.desired_wrench.torque.y - feedback.actual_wrench.torque.y;
    feedback.error_wrench.torque.z = feedback.desired_wrench.torque.z - feedback.actual_wrench.torque.z;

    action_server_->publishFeedback(feedback);
}

} // namespace direct_force_controller

PLUGINLIB_EXPORT_CLASS(direct_force_controller::DirectForceController, controller_interface::ControllerBase)