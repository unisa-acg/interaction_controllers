/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gazebo_force_torque_sensor.cpp
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    May 13, 2020
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <force_torque_sensor/exceptions.h>
#include <force_torque_sensor/gazebo_force_torque_sensor.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

using namespace force_torque_sensor;

ForceTorqueSensorGazebo::ForceTorqueSensorGazebo()
  : ForceTorqueSensor()
  , force_torque_bias_(number_of_channels)
  , number_of_samples_for_bias_(ForceTorqueSensorGazebo::default_number_of_samples_for_bias)
{
  ros::NodeHandle nh;

  sensor_topic_subscriber_ =
      nh.subscribe("gazebo_ft_sensor", 1, &force_torque_sensor::ForceTorqueSensorGazebo::receiveWrenchMessage_, this);
  ROS_INFO("Force/torque sensor subscribed to the topic /gazebo_ft_sensor");

  computeBias();
}

ForceTorqueSensorGazebo::~ForceTorqueSensorGazebo()
{
}

void ForceTorqueSensorGazebo::getMeasure(std::vector<float>& force_torque) const
{
  // Callback "receiveWrenchMessage_" processing is synchronous with this
  // method. If more than one message is received between two spinOnce calls,
  // all of them are discarded, but the last one.
  ros::spinOnce();

  force_torque.resize(number_of_channels);
  force_torque[0] = wrench_msg_.force.x - force_torque_bias_[0];
  force_torque[1] = wrench_msg_.force.y - force_torque_bias_[1];
  force_torque[2] = wrench_msg_.force.z - force_torque_bias_[2];
  force_torque[3] = wrench_msg_.torque.x - force_torque_bias_[3];
  force_torque[4] = wrench_msg_.torque.y - force_torque_bias_[4];
  force_torque[5] = wrench_msg_.torque.z - force_torque_bias_[5];
}

void ForceTorqueSensorGazebo::receiveWrenchMessage_(const geometry_msgs::WrenchStamped& wrench_msg)
{
  wrench_msg_ = wrench_msg.wrench;
}

void ForceTorqueSensorGazebo::computeBias()
{
  std::vector<float> force_torque(number_of_channels, 0);
  std::vector<float> previous_reading(number_of_channels, 0);
  std::fill(force_torque_bias_.begin(), force_torque_bias_.end(), 0);
  float tolerance = 1e-6;
  int stable_readings = 0;
  int max_reads = number_of_samples_for_bias_ + 10;

  // Loop until reaching a maximum number of readings or the desired number of
  // stable readings
  for (int i = 0; i < max_reads && stable_readings < number_of_samples_for_bias_; i++)
  {
    getMeasure(force_torque);

    bool non_zero_measure = false;
    bool equal_to_previous = true;

    for (int j = 0; j < number_of_channels; j++)
    {
      // Check if at least one measure is beyond tolerance (non-zero)
      // If yes, this reading is a candidate stable reading
      if (fabs(force_torque[j]) > tolerance)
        non_zero_measure = true;

      // In order for a reading to be stable, all measures should be equal to
      // the previous ones
      equal_to_previous = equal_to_previous & (fabs(force_torque[j] - previous_reading[j]) < tolerance);
    }

    if (non_zero_measure && equal_to_previous)
      stable_readings++;
    else
      stable_readings = 0;

    previous_reading = force_torque;

    // Sleep 100ms to let the publisher deliver new readings
    usleep(1e5);
  }

  // Throw an exception if we did not reach the desired number of stable
  // readings
  if (stable_readings < number_of_samples_for_bias_)
    throw force_torque_sensor::Exception("Cannot retrieve stable measures from the F/T sensor");

  // Update the bias
  force_torque_bias_ = force_torque;

  ROS_INFO("Force/torque sensor bias set to return null measures");
}

PLUGINLIB_EXPORT_CLASS(force_torque_sensor::ForceTorqueSensorGazebo, force_torque_sensor::ForceTorqueSensor)
