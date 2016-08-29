/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "PIRegulator.h"

#include <algorithm>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

PIRegulator::PIRegulator(double const kP, double const kI, double const dt_s, double const min, double const max)
: _kP(kP),
  _kI(kI),
  _dt_s(dt_s),
  _min(min),
  _max(max),
  _integral(0.0)
{

}

double PIRegulator::calc(double const target_value, double const actual_value)
{
  double const error = target_value - actual_value;

  double const p_out = _kP * error;

  double const integral = _integral + error * _dt_s;
  if(integral >= _min && integral <= _max)
  {
    _integral = integral;
  }
  
  double const i_out = _kI * _integral;

  double out = p_out + i_out;

  out = std::max<double>(out, _min);
  out = std::min<double>(out, _max);

  return out;
}
