/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   exceptions.h
 * Author:  Sabatino Simeone
 * Org.:    UNISA
 * Date:    Dec 6, 2019
 *
 * This module masks the ros::Exception class in the Exception
 * class of this namespace/package.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_ADMITTANCE_CONTROLLER_EXCEPTIONS_H_
#define INCLUDE_ADMITTANCE_CONTROLLER_EXCEPTIONS_H_

#include <ros/exception.h>

namespace admittance_controller
{
typedef ros::Exception Exception;
}

#endif /* INCLUDE_ADMITTANCE_CONTROLLER_EXCEPTIONS_H_ */