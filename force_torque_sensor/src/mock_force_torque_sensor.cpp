/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   mock_force_torque_sensor.cpp
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    May 11, 2020
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <force_torque_sensor/mock_force_torque_sensor.h>
#include <pluginlib/class_list_macros.h>

using namespace force_torque_sensor;

ForceTorqueSensorMock::ForceTorqueSensorMock() : ForceTorqueSensor()
{
}

ForceTorqueSensorMock::~ForceTorqueSensorMock()
{
}

void ForceTorqueSensorMock::getMeasure(std::vector<float>& force_torque) const
{
  for (int i = 0; i < number_of_channels; i++)
  {
    force_torque[i] = ((double)rand() / (RAND_MAX));
  }
}

void ForceTorqueSensorMock::computeBias()
{
}

PLUGINLIB_EXPORT_CLASS(force_torque_sensor::ForceTorqueSensorMock, force_torque_sensor::ForceTorqueSensor)
