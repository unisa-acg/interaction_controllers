/* -------------------------------------------------------------------
 *
 * This module has been developed as part of a collaboration between
 * the Automatic Control Group @ UNISA and ALTEC.
 *
 * Title:   exceptions.h
 * Author:  Giovanni Longobardi
 * Org.:    UNISA
 * Date:    Oct 2, 2020
 *
 * For the time being, the Exception class of the
 * workspace_trajectory namespace/package masks the
 * moveit::Exception class, as all the packages of moveit use it.
 *
 * -------------------------------------------------------------------
 */

#ifndef INCLUDE_WORKSPACE_TRAJECTORY_EXCEPTIONS_H_
#define INCLUDE_WORKSPACE_TRAJECTORY_EXCEPTIONS_H_

#include <moveit/exceptions/exceptions.h>

namespace workspace_trajectory
{

typedef moveit::Exception Exception;

}

#endif /* INCLUDE_WORKSPACE_TRAJECTORY_EXCEPTIONS_H_ */