/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Odometry.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

double Odometry::calcLinearSpeed_m_per_s(double const wheel_distance_m, double const speed_left_m_per_s, double const speed_right_m_per_s)
{
  double const linear_speed_m_per_s = (speed_left_m_per_s + speed_right_m_per_s) / 2.0;
  return linear_speed_m_per_s;
}
