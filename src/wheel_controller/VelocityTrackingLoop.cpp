/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/* Theory: https://www.embeddedrelated.com/showarticle/530.php */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "VelocityTrackingLoop.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

VelocityTrackingLoop::VelocityTrackingLoop(double const kP, double const kI)
:
  _kP   (kP),
  _kI   (kI),
  _velocity_estimated (0.0),
  _velocity_integrated(0.0),
  _position_estimated (0.0)
{

}

void VelocityTrackingLoop::update(double const position_measured, double const dt_s)
{
  _position_estimated         += _velocity_estimated * dt_s;

  double const position_error  = position_measured - _position_estimated;

  _velocity_integrated        += position_error * _kI * dt_s;
  _velocity_estimated          = position_error * _kP + _velocity_integrated;
}

float VelocityTrackingLoop::getVelocity() const
{
  return _velocity_estimated;
}
