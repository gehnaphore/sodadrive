/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "EMG49.h"

#define USE_MATH_DEFINES
#include <math.h>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const ENCODER_TICKS_PER_SINGLE_MOTOR_REVOLUTION     = 588;
static double const MOTOR_REVOLUTIONS_PER_SINGLE_WHEEL_REVOLUTION = 49;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

double EMG49::calcDistanceTraveled_m(uint32_t const delta_encoder_ticks, double const wheel_diameter_m)
{
  double const distance_traveled_m =
      (
          (
              static_cast<double>(delta_encoder_ticks) / ENCODER_TICKS_PER_SINGLE_MOTOR_REVOLUTION
          )
          /
          MOTOR_REVOLUTIONS_PER_SINGLE_WHEEL_REVOLUTION
      )
      *
      (
          wheel_diameter_m * M_PI
      );

  return distance_traveled_m;
}
