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

#ifndef INCLUDE_DIRECT_FORCE_CONTROLLER_EXCEPTIONS_H_
#define INCLUDE_DIRECT_FORCE_CONTROLLER_EXCEPTIONS_H_

#include <ros/exception.h>

namespace direct_force_controller
{
typedef ros::Exception Exception;
}

#endif /* INCLUDE_DIRECT_FORCE_CONTROLLER_EXCEPTIONS_H_ */