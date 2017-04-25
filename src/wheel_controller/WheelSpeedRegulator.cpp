/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "WheelSpeedRegulator.h"

#include <limits>
#include <ros/ros.h>

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

WheelSpeedRegulator::WheelSpeedRegulator(double const kP, double const kI, double const kD, double const dt_s)
: _speed_regulator           (kP, kI, kD, dt_s, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max()),
  _speed_m_per_s_target_value(0.0),
  _speed                     (0)
{

}

void WheelSpeedRegulator::setSpeedTargetValue(double const speed_m_per_s)
{
  _speed_m_per_s_target_value = speed_m_per_s;
  ROS_INFO("(%08x) target speed = %lf",this,_speed_m_per_s_target_value);
}

void WheelSpeedRegulator::updateWithActualValue(double const speed_m_per_s)
{
  _speed = static_cast<int8_t>(_speed_regulator.calc(_speed_m_per_s_target_value, speed_m_per_s));
  ROS_INFO("(%08x) actual speed = %lf",this,speed_m_per_s);
  ROS_INFO("(%08x) speed value  = %d",this,(int)_speed);
}

int8_t WheelSpeedRegulator::getSpeed() const
{
  return _speed;
}
