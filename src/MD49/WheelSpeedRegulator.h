/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_WHEELSPEEDREGULATOR_H_
#define RPI_SRC_MD49_WHEELSPEEDREGULATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include "PIRegulator.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class WheelSpeedRegulator
{

public:

  WheelSpeedRegulator           (double const kP, double const kI, double const dt_s);

  void    setSpeedTargetValue   (double const speed_m_per_s );
  void    updateWithActualValue (double const speed_m_per_s );
  int8_t  getSpeed              (                           ) const;

private:

  PIRegulator   _speed_regulator;
  double        _speed_m_per_s_target_value;
  int8_t        _speed;

};

#endif /* RPI_SRC_MD49_WHEELSPEEDREGULATOR_H_ */
