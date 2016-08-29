/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/

#include "LoopPeriodAnalyser.h"

/************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 ************************************************************************************/

LoopPeriodAnalyser::LoopPeriodAnalyser() : _loop_period_s(0.0)
{

}

/// \brief this method must be called ONLY ONCE per loop and is responsible for measuring the loop period
void LoopPeriodAnalyser::update()
{
  static bool is_first_loop_run = true;

  _now = boost::posix_time::microsec_clock::local_time();

  if(!is_first_loop_run)
    {
      boost::posix_time::time_duration const time_duration = _now - _prev;

      _loop_period_s = time_duration.total_milliseconds() / 1000.0;
    }
  else
    {
      is_first_loop_run = false;
    }

  _prev = _now;
}

float LoopPeriodAnalyser::getLoopPeriodInSec()
{
  return _loop_period_s;
}
