/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   admittance_controller.h
 * Author:  Sabatino Simeone, Giovanni Longobardi
 * Org.:    UNISA
 * Date:    Dec 6, 2019
 *
 * This class implements an admittance controller with inner position
 * loop based on ros_control
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_ADMITTANCE_CONTROLLER_ADMITTANCE_CONTROLLER_H
#define INCLUDE_ADMITTANCE_CONTROLLER_ADMITTANCE_CONTROLLER_H

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <force_torque_sensor/force_torque_sensor.h>
#include <acg_control_msg/FollowWorkspaceTrajectoryAction.h>

namespace admittance_controller
{

class AdmittanceController : public controller_interface::Controller<hardware_interface::PositionJointInterface>

{
  /**
   * @brief Enumerator defining the states of the admittance controller
   */
  enum State
  {
    independent_joint_control,
    static_admittance_control,
    trajectory_execution_admittance_control
  };

public:
  static const unsigned int x = 0;
  static const unsigned int y = 1;
  static const unsigned int z = 2;
  static const unsigned int admittance_control_dof = 3;

  AdmittanceController();
  virtual ~AdmittanceController();

  bool init(hardware_interface::PositionJointInterface* robot_hw, ros::NodeHandle & controller_nh);

  void starting(const ros::Time & time);

  /**
    * @throw admittance_controller::Exception if no valid solution to the kinematic inversion is found
    */
  void update(const ros::Time & time, const ros::Duration & period);

  void stopping(const ros::Time &);

  // Refer to Eigen documentation to understand this macro.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
private:
  /**
    * @brief check if the desired joint positions are reached considering a given position tolerance.
    *
    * @param joint_values desired joint positions that have to be compared with the actual joint positions
    * @return true if all joint positions are reached, false otherwise
    */
  bool isPositionReached_(const std::vector<double> & joint_values) const;

  /**
    * @brief compute the inverse kinematics
    *
    * @param pose is the transformation matrix corresponding to the desired end-effector pose
    * @param joint_values are the joint values resulting from the kinematic inversion
    * @return true if a valid solution to the inverse kinematics is found, false otherwise
    */
  bool computeInverseKinematics_(Eigen::Isometry3d pose, std::vector<double> & joint_values);
  
  /**
    * @brief Compute the difference between two joint values (angles or linear displacement)
    * 
    * @param diff Computed vector difference.
    * @param minuend First vector.
    * @param subtrahend Second vector.
    * @param joint_model_group joint model group providing information on the type of joints (e.g. revolute, prismatic)
    * 
    * @throw admittance_controller::Exception if the JMG contains joint types that are not currently supported
    */
  void computeVectorsDifference_(std::vector<double> & diff, 
                                 const std::vector<double> & minuend, 
                                 const std::vector<double> & subtrahend,
                                 const moveit::core::JointModelGroup* joint_model_group) const;

  /**
    * @brief callback called when a new goal is received from the action client
    */
  void getActionGoal_();

  /**
    * @brief update the feedback and send it to the action client during the goal execution
    */
  void updateActionFeedback_();

  std::unique_ptr<pluginlib::ClassLoader<force_torque_sensor::ForceTorqueSensor>> force_torque_sensor_loader_;
  boost::shared_ptr<force_torque_sensor::ForceTorqueSensor> force_torque_sensor_;

  State active_state_;
  bool trajectory_received_;
  double ik_timeout_;
  double sp_tolerance_;
  double ik_tolerance_;

  const moveit::core::JointModelGroup *joint_model_group_;
  std::shared_ptr<const robot_model_loader::RobotModelLoader> robot_model_loader_;
  std::shared_ptr<const robot_model::RobotModel> kinematic_model_;
  std::shared_ptr<robot_state::RobotState> kinematic_state_;

  std::string base_frame_name_;
  Eigen::Isometry3d base_to_end_effector_transform_; // Transformation matrix at the setpoint kept fixed during the trajectory execution
  Eigen::Isometry3d world_to_base_transform_;
  Eigen::Isometry3d end_effector_to_ft_sensor_transform_;

  std::vector<hardware_interface::JointHandle> joint_position_handles_;
  std::vector<double> actual_joint_positions_;
  std::vector<double> joint_positions_command_;
  unsigned int number_of_joints_;
  ros::Time first_timestamp_;

  // Admittance control
  Eigen::Matrix3d M_d_inv_; // Mass matrix
  Eigen::Matrix3d K_D_; // Damping Matrix
  Eigen::Matrix3d K_P_; // Stiffness matrix

  Eigen::Vector3d f_d_;
  Eigen::Vector3d f_e_;
  Eigen::Vector3d delta_f_;
  Eigen::Isometry3d x_c_b_; // Workspace command in the base frame
  Eigen::Vector3d x_c_; // Workspace command computed by the control law expressed in the end-effector frame
  Eigen::Vector3d x_d_b_; // Desired end-effector position expressed in the base frame
  Eigen::Vector3d x_d_e_; // Desired end-effector position expressed in the end-effector frame with respect to the setpoint position
  Eigen::Vector3d x_dot_c_; // End-effector velocity oriented according to the end-effector frame
  Eigen::Vector3d x_dot_dot_c_; // Control law

  // action server
  std::shared_ptr<actionlib::SimpleActionServer<acg_control_msg::FollowWorkspaceTrajectoryAction>> action_server_;
  acg_control_msg::FollowWorkspaceTrajectoryResult result_;
  workspace_trajectory_msgs::WorkspaceTrajectory reference_ws_trajectory_;

  int number_reached_trajectory_points_;
};

} // namespace admittance_controller

#endif /* INCLUDE_ADMITTANCE_CONTROLLER_ADMITTANCE_CONTROLLER_H */
