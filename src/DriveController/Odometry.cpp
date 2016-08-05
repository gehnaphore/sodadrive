/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Odometry.h"

#define USE_MATH_DEFINES
#include <math.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

Odometry::Odometry(double const wheel_distance_m)
: _wheel_distance_m(wheel_distance_m)
{

}

double Odometry::calcLinearSpeed(double const speed_left_m_per_s, double const speed_right_m_per_s)
{
  double const linear_speed_m_per_s = (speed_left_m_per_s + speed_right_m_per_s) / 2.0;
  return linear_speed_m_per_s;
}

double Odometry::calcAngularSpeed (double const speed_left_m_per_s, double const speed_right_m_per_s)
{
  double const angular_speed_rad_per_s = (speed_left_m_per_s + speed_right_m_per_s) / _wheel_distance_m;
  double const angular_speed_deg_per_s = angular_speed_rad_per_s * 180.0 / M_PI;
  return angular_speed_deg_per_s;
}
