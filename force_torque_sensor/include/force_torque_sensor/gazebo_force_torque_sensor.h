/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gazebo_force_torque_sensor.h
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    May 13, 2020
 *
 * The force torque sensor is loaded directly into the Gazebo
 * environment through the use of a plugin.
 * This class contains an implementation of the ForceTorqueSensor
 * interface to access sensor data through ROS topics communication.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_FORCE_TORQUE_SENSOR_GAZEBO_FORCE_TORQUE_SENSOR_H_
#define INCLUDE_FORCE_TORQUE_SENSOR_GAZEBO_FORCE_TORQUE_SENSOR_H_

#include <force_torque_sensor/force_torque_sensor.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

namespace force_torque_sensor
{
class ForceTorqueSensorGazebo : public ForceTorqueSensor
{
public:
  static const unsigned int default_number_of_samples_for_bias = 10;

  /**
   * @brief Construct a ForceTorqueSensorGazebo object
   */
  ForceTorqueSensorGazebo();

  /**
   * @brief Destruct a ForceTorqueSensorGazebo object
   */
  ~ForceTorqueSensorGazebo();

  /**
   * @brief Refer to the superclass documentation.
   */
  void getMeasure(std::vector<float>& force_torque) const;

  /**
   * @brief Refer to the superclass documentation.
   *
   * @throw force_torque_sensor::Exception if the bias cannot be computed
   * because of unstable measures
   */
  void computeBias();

private:
  /**
   * @brief This is the callback function that will get called when a new
   * message is received on the topic.
   *
   * @param wrench_msg is the message received on the topic.
   */
  void receiveWrenchMessage_(const geometry_msgs::WrenchStamped& wrench_msg);

  ros::Subscriber sensor_topic_subscriber_;
  unsigned int number_of_samples_for_bias_;
  std::vector<float> force_torque_bias_;
  geometry_msgs::Wrench wrench_msg_;
};

}  // namespace force_torque_sensor

#endif /* INCLUDE_FORCE_TORQUE_SENSOR_GAZEBO_FORCE_TORQUE_SENSOR_H_ */
