/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_EMG49_H_
#define RPI_SRC_MD49_EMG49_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class EMG49
{

public:

  static double calcDistanceTraveled_m(int32_t const delta_encoder_ticks, double const wheel_diameter_m);

private:

  EMG49() { }
  EMG49(EMG49 const &other) { }

};

#endif /* RPI_SRC_MD49_EMG49_H_ */
