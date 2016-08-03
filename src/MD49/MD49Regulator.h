/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_MD49REGULATOR_H_
#define RPI_SRC_MD49_MD49REGULATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include "PIRegulator.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MD49Regulator
{

public:

  MD49Regulator(double const kP_1, double const kI_1,
                double const kP_2, double const kI_2,
                double const dt_s);

  void    setSpeed1TargetValue (double const speed_1_m_per_s);
  void    setSpeed2TargetValue (double const speed_2_m_per_s);

  void    updateWithActualValue(double const speed_1_m_per_s,
                                double const speed_2_m_per_s);

  int8_t  getSpeed1            (                            ) const;
  int8_t  getSpeed2            (                            ) const;

private:

  PIRegulator _speed_1_regulator,
              _speed_2_regulator;

  double      _speed_1_m_per_s_target_value,
              _speed_2_m_per_s_target_value;

  int8_t      _speed_1,
              _speed_2;

};


#endif /* RPI_SRC_MD49_MD49REGULATOR_H_ */
