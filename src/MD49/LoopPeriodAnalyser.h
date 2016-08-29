/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef LOOPPERIODANALYSER_H_
#define LOOPPERIODANALYSER_H_

/************************************************************************************
 * INCLUDES
 ************************************************************************************/

#include <boost/date_time/posix_time/posix_time.hpp>

/************************************************************************************
 * CLASS DECLARATION
 ************************************************************************************/

class LoopPeriodAnalyser
{

public:

  LoopPeriodAnalyser();

  ///   \brief this method must be called ONLY ONCE per loop and is responsible for measuring the loop period
  void  update();

  float getLoopPeriodInSec();

private:

  float                     _loop_period_s;

  boost::posix_time::ptime  _now,
                            _prev;

};

#endif /* LOOPPERIODANALYSER_H_ */
