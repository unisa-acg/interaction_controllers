/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   mock_force_torque_sensor.h
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    May 11, 2020
 *
 * This class contains an implementation of the ForceTorqueSensor
 * interface to support testing of components that use F/T sensors,
 * but do not rely on F/T sensor readings.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_FORCE_TORQUE_SENSOR_MOCK_FORCE_TORQUE_SENSOR_H_
#define INCLUDE_FORCE_TORQUE_SENSOR_MOCK_FORCE_TORQUE_SENSOR_H_

#include <force_torque_sensor/force_torque_sensor.h>

namespace force_torque_sensor
{
class ForceTorqueSensorMock : public ForceTorqueSensor
{
public:
  /**
   * @brief Construct a ForceTorqueSensorMock object
   */
  ForceTorqueSensorMock();

  /**
   * @brief Destruct a ForceTorqueSensorMock object
   */
  ~ForceTorqueSensorMock();

  /**
   * @brief Refer to the superclass documentation.
   */
  void getMeasure(std::vector<float>& force_torque) const;

  /**
   * @brief Refer to the superclass documentation.
   */
  void computeBias();
};

}  // namespace force_torque_sensor

#endif /* INCLUDE_FORCE_TORQUE_SENSOR_MOCK_FORCE_TORQUE_SENSOR_H_ */
