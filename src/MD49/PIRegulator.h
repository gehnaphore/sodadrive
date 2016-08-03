/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_PIREGULATOR_H_
#define RPI_SRC_MD49_PIREGULATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class PIRegulator
{

public:

  PIRegulator(double const kP, double const kI, double const dt_s, double const min, double const max);

  double calc(double const target_value, double const actual_value);

private:

  double _kP, _kI, _dt_s, _min, _max,
         _integral;

};

#endif /* RPI_SRC_MD49_PIREGULATOR_H_ */
