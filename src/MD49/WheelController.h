/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_WHEELCONTROLLER_H_
#define RPI_SRC_MD49_WHEELCONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <string>

#include "MD49.h"
#include "WheelSpeedRegulator.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class WheelController
{

public:

  static double const T_LOOP_UPDATE_s  = 0.1;    /* 100 ms */

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

private:

  MD49                * _md49;

  WheelSpeedRegulator   _speed_1_regulator;
  WheelSpeedRegulator   _speed_2_regulator;

  void getDeltaEncoderValues    (int32_t      & delta_encoder_1,
                                 int32_t      & delta_encoder_2);

  void calcActualSpeed_m_per_s  (int32_t const   delta_encoder_1,
                                 int32_t const   delta_encoder_2,
                                 double        & actual_speed_1_m_per_s,
                                 double        & actual_speed_2_m_per_s);
};

#endif /* RPI_SRC_MD49_WHEELCONTROLLER_H_ */
