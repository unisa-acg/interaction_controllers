/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   demo_force_torque_sensor_read.cpp
 * Author:  Giovanni Longobardi, Sabatino Simeone
 * Org.:    UNISA
 * Date:    May 15, 2020
 *
 * In this demo, a plugin force torque sensor is initialized to get
 * some measures that are printed to standard output.
 * The specific sensor is taken from the parameter server.
 *
 * -------------------------------------------------------------------
 */

#include <force_torque_sensor/exceptions.h>
#include <force_torque_sensor/force_torque_sensor.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

using namespace force_torque_sensor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_force_torque_sensor");
  ros::NodeHandle nh;
  boost::shared_ptr<force_torque_sensor::ForceTorqueSensor> ft_sensor;

  std::string force_torque_sensor;
  if (!nh.getParam("/force_torque_sensor_type", force_torque_sensor))
    throw force_torque_sensor::Exception("Cannot find sensor type on the parameter server");

  pluginlib::ClassLoader<force_torque_sensor::ForceTorqueSensor> sensor_loader(
      "force_torque_sensor", "force_torque_sensor::ForceTorqueSensor");

  try
  {
    ft_sensor = sensor_loader.createInstance("force_torque_sensor::" + force_torque_sensor);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
  }

  std::vector<float> force_torque(ForceTorqueSensor::number_of_channels, 0);
  int microseconds = 1e6;

  while (ros::ok())
  {
    ft_sensor->getAverageMeasure(force_torque);
    force_torque_sensor::ForceTorqueSensor::printMeasure(force_torque);
    usleep(microseconds);
  }

  ft_sensor.reset();

  ros::shutdown;
  return 0;
}
