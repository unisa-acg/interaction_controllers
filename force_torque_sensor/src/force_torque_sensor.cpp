/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   force_torque_sensor.cpp
 * Author:  Sabatino Simeone
 * Org.:    UNISA
 * Date:    Oct 30, 2019
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <force_torque_sensor/exceptions.h>
#include <force_torque_sensor/force_torque_sensor.h>
#include <ros/console.h>

using namespace force_torque_sensor;

ForceTorqueSensor::ForceTorqueSensor()
  : number_of_samples_for_average_measure_(ForceTorqueSensor::default_number_samples_average_measure)
{
}

void ForceTorqueSensor::getAverageMeasure(std::vector<float>& force_torque) const
{
  int c, i;
  float sum[number_of_channels] = { 0, 0, 0, 0, 0, 0 };

  for (c = 0; c < number_of_samples_for_average_measure_; c++)
  {
    getMeasure(force_torque);

    for (i = 0; i < number_of_channels; i++)
      sum[i] += force_torque[i];
  }

  for (i = 0; i < number_of_channels; i++)
    force_torque[i] = sum[i] / number_of_samples_for_average_measure_;
}

void ForceTorqueSensor::printMeasure(const std::vector<float>& force_torque)
{
  ROS_INFO("\nFx %f Fy %f Fz %f Tx %f Ty %f Tz %f ", force_torque[0], force_torque[1], force_torque[2], force_torque[3],
           force_torque[4], force_torque[5]);
}

void ForceTorqueSensor::setNumberOfSamplesForAverageMeasure(unsigned int number_samples)
{
  if (number_samples >= 1)
    number_of_samples_for_average_measure_ = number_samples;
  else
    throw force_torque_sensor::Exception("Number of samples must be bigger than 0");
}
