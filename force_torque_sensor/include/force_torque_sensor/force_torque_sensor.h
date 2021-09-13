/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   force_torque_sensor.h
 * Author:  Sabatino Simeone
 * Org.:    UNISA
 * Date:    Oct 30, 2019
 *
 * This class is an abstract interface providing methods to read data
 * from F/T sensors.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_FORCE_TORQUE_SENSOR_FORCE_TORQUE_SENSOR_H_
#define INCLUDE_FORCE_TORQUE_SENSOR_FORCE_TORQUE_SENSOR_H_

#include <vector>

namespace force_torque_sensor
{
class ForceTorqueSensor
{
public:
  static const unsigned int number_of_channels = 6;
  static const unsigned int default_number_samples_average_measure = 10;

  /**
   * @brief Print force/torque vector
   *
   * @param force_torque vector containing the results of the measure
   */
  static void printMeasure(const std::vector<float>& force_torque);

  /**
   * @brief Construct a ForceTorqueSensor object
   */
  ForceTorqueSensor();

  /**
   * @brief Read force and torque data measured from the sensor
   *
   * @param force_torque vector containing the results of the measure
   */
  virtual void getMeasure(std::vector<float>& force_torque) const = 0;

  /**
   * @brief Read a defined number of samples from the sensor and compute the
   * bias
   */
  virtual void computeBias() = 0;

  /**
   * @brief Read a pre-defined number of samples from the sensor and then take
   * the average of these
   *
   * @param force_torque vector containing the results of the measure
   */
  void getAverageMeasure(std::vector<float>& force_torque) const;

  /**
   * @brief Set the number of samples for a single force/torque measure when
   * getAverageMeasure is used
   *
   * @param number_samples number of samples used for average
   * @throw force_torque_sensor::Exception if number_samples is zero
   */
  void setNumberOfSamplesForAverageMeasure(unsigned int number_samples);

protected:
  unsigned int number_of_samples_for_average_measure_;
};

}  // namespace force_torque_sensor

#endif /* INCLUDE_FORCE_TORQUE_SENSOR_FORCE_TORQUE_SENSOR_H_ */
