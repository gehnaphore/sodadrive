/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "WheelController.h"

#include <assert.h>

#include "EMG49.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const kP_speed_1        = 0.0;
static double const kI_speed_1        = 150.0;
static double const kD_speed_1        = 0.0;
static double const kP_speed_2        = kP_speed_1;
static double const kI_speed_2        = kI_speed_1;
static double const kD_speed_2        = kD_speed_1;

static double const WHEEL_DIAMETER_m  = 0.125;

static double const MIN_SPEED_m_per_s = 0.03;

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

WheelController::WheelController(MD49 *md49)
:
  _md49(md49),
  _speed_1_regulator (kP_speed_1, kI_speed_1, kD_speed_1, T_LOOP_UPDATE_s),
  _speed_2_regulator (kP_speed_2, kI_speed_2, kD_speed_2, T_LOOP_UPDATE_s),
  last_encoder_1(0),
  last_encoder_2(0)  
{
  assert(_md49 != 0);
}

void WheelController::run(sIn const &in, sOut *out)
{
  assert(out != 0);

  if(fabs(in.speed_1_m_per_s_target_value) >= MIN_SPEED_m_per_s)  _speed_1_regulator.setSpeedTargetValue(in.speed_1_m_per_s_target_value);
  else                                               _speed_1_regulator.setSpeedTargetValue(0.0);
  if(fabs(in.speed_2_m_per_s_target_value) >= MIN_SPEED_m_per_s)  _speed_2_regulator.setSpeedTargetValue(in.speed_2_m_per_s_target_value);
  else                                               _speed_2_regulator.setSpeedTargetValue(0.0);

  int32_t delta_encoder_1 = 0,
          delta_encoder_2 = 0;

  getDeltaEncoderValues     (delta_encoder_1,
                             delta_encoder_2);

  _loop_period_analyser.update();

  double actual_speed_1_m_per_s = 0.0,
         actual_speed_2_m_per_s = 0.0;

  calcActualSpeed_m_per_s   (delta_encoder_1,
                             delta_encoder_2,
                             _loop_period_analyser.getLoopPeriodInSec(),
                             actual_speed_1_m_per_s,
                             actual_speed_2_m_per_s);

  _speed_1_regulator.updateWithActualValue(actual_speed_1_m_per_s);
  _speed_2_regulator.updateWithActualValue(actual_speed_2_m_per_s);

  int8_t  speed_1 = _speed_1_regulator.getSpeed();
  int8_t  speed_2 = _speed_2_regulator.getSpeed();

  if ((speed_1 > 0) && (speed_1 < 5)) {
    speed_1 = 5;
  } else if ((speed_1 < 0) && (speed_1 > -5)) {
    speed_1 = -5;
  }

  if ((speed_2 > 0) && (speed_2 < 5)) {
    speed_2 = 5;
  } else if ((speed_2 < 0) && (speed_2 > -5)) {
    speed_2 = -5;
  }

  _md49->setSpeed1(speed_1);
  _md49->setSpeed2(speed_2);

  out->speed_1_m_per_s_actual_value = actual_speed_1_m_per_s;
  out->speed_2_m_per_s_actual_value = actual_speed_2_m_per_s;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void WheelController::getLastEncoderValues(int32_t & encoder_1,
                                           int32_t & encoder_2) {
  encoder_1 = last_encoder_1;                                             
  encoder_2 = last_encoder_2;                                             
}

void WheelController::getDeltaEncoderValues(int32_t & delta_encoder_1,
                                            int32_t & delta_encoder_2)
{
         int32_t current_encoder_1 = 0,
                 current_encoder_2 = 0;

  _md49->getEncoders(current_encoder_1, current_encoder_2);

  delta_encoder_1 = current_encoder_1 - last_encoder_1;
  delta_encoder_2 = current_encoder_2 - last_encoder_2;

  last_encoder_1  = current_encoder_1;
  last_encoder_2  = current_encoder_2;
}

void WheelController::calcActualSpeed_m_per_s(int32_t const   delta_encoder_1,
                                              int32_t const   delta_encoder_2,
                                              double  const   loop_durations_s,
                                              double        & actual_speed_1_m_per_s,
                                              double        & actual_speed_2_m_per_s)
{
  actual_speed_1_m_per_s = EMG49::calcDistanceTraveled_m(delta_encoder_1, WHEEL_DIAMETER_m) / loop_durations_s;
  actual_speed_2_m_per_s = EMG49::calcDistanceTraveled_m(delta_encoder_2, WHEEL_DIAMETER_m) / loop_durations_s;
}
