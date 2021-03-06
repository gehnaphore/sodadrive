/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_WHEEL_CONTROLLER_WHEELCONTROLLER_H_
#define RPI_SRC_WHEEL_CONTROLLER_WHEELCONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <string>

#include "MD49.h"
#include "LoopPeriodAnalyser.h"
#include "WheelSpeedRegulator.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class WheelController
{

public:

  static double const T_LOOP_UPDATE_s  = 0.20;

  WheelController(MD49 *md49);

  typedef struct
  {
    float speed_1_m_per_s_target_value;
    float speed_2_m_per_s_target_value;
  } sIn;

  typedef struct
  {
    float speed_1_m_per_s_actual_value;
    float speed_2_m_per_s_actual_value;
  } sOut;

  void run(sIn const &in, sOut *out);

  void getLastEncoderValues    (int32_t      & encoder_1,
                                int32_t      & encoder_2);


private:

  MD49                * _md49;

  WheelSpeedRegulator   _speed_1_regulator;
  WheelSpeedRegulator   _speed_2_regulator;
  int32_t last_encoder_1;
  int32_t last_encoder_2;

  LoopPeriodAnalyser    _loop_period_analyser;

  void getDeltaEncoderValues    (int32_t      & delta_encoder_1,
                                 int32_t      & delta_encoder_2);

  void calcActualSpeed_m_per_s  (int32_t const   delta_encoder_1,
                                 int32_t const   delta_encoder_2,
                                 double  const   loop_durations_s,
                                 double        & actual_speed_1_m_per_s,
                                 double        & actual_speed_2_m_per_s);
};

#endif /* RPI_SRC_WHEEL_CONTROLLER_WHEELCONTROLLER_H_ */
