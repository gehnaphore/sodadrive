/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_WHEEL_CONTROLLER_PIDREGULATOR_H_
#define RPI_SRC_WHEEL_CONTROLLER_PIDREGULATOR_H_

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class PIDRegulator
{

public:

  PIDRegulator(double const kP, double const kI, double const kD, double const dt_s, double const min, double const max);

  double calc (double const target_value, double const actual_value);

private:

  double _kP, _kI, _kD, _dt_s, _min, _max,
         _error_sum,
         _error_prev;

};

#endif /* RPI_SRC_WHEEL_CONTROLLER_PIDREGULATOR_H_ */
