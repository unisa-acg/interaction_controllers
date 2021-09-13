/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   exceptions.h
 * Author:  Sabatino Simeone
 * Org.:    UNISA
 * Date:    Oct 30, 2019
 *
 * This module masks the ros::Exception class in the Exception
 * class of this namespace/package.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_FORCE_TORQUE_SENSOR_EXCEPTIONS_H_
#define INCLUDE_FORCE_TORQUE_SENSOR_EXCEPTIONS_H_

#include <ros/exception.h>

namespace force_torque_sensor
{
typedef ros::Exception Exception;
}

#endif /* INCLUDE_FORCE_TORQUE_SENSOR_EXCEPTIONS_H_ */