/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "PIDRegulator.h"

#include <algorithm>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

PIDRegulator::PIDRegulator(double const kP, double const kI, double const kD, double const dt_s, double const min, double const max)
: _kP(kP),
  _kI(kI),
  _kD(kD),
  _dt_s(dt_s),
  _min(min),
  _max(max),
  _error_sum(0.0),
  _error_prev(0.0)
{

}

double PIDRegulator::calc(double const target_value, double const actual_value)
{
  double const error      = target_value - actual_value;

  /* P */

  double const p_out      = _kP * error;

  /* I */

  double const error_sum  = _error_sum + error * _dt_s;
  double       i_out      = _kI * error_sum;

  if(i_out >= _min && i_out <= _max)
  {
    _error_sum = error_sum;
  }
  else
  {
    i_out = std::max<double>(i_out, _min);
    i_out = std::min<double>(i_out, _max);
  }

  /* D */

  double const d_out      = _kD * (_error_prev - error);
  _error_prev             = error;

  /* Limit regulator output */

  double       out        = p_out + i_out + d_out;

  out = std::max<double>(out, _min);
  out = std::min<double>(out, _max);

  return out;
}
