/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "DriveController.h"

#include <assert.h>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const WHEEL_DISTANCE_m = 0.34;

static double const kP_LINEAR        = 0.50;
static double const kI_LINEAR        = 2.00;
static double const kP_ANGULAR       = 0.00;
static double const kI_ANGULAR       = 0.01;

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

DriveController::DriveController()
:
    _odometry(WHEEL_DISTANCE_m),
    _differential_drive_regulator(kP_LINEAR,
                                  kI_LINEAR,
                                  kP_ANGULAR,
                                  kI_ANGULAR,
                                  T_LOOP_UPDATE_s)
{

}

void DriveController::run(sIn const &in, sOut *out)
{
  assert(out != 0);

  _differential_drive_regulator.setLinearX  (in.linear_x_m_per_s_target_value);
  _differential_drive_regulator.setAngularZ (in.angular_z_deg_per_s_target_value);

  double const linear_x_m_per_s_actual_value        = _odometry.calcLinearSpeed (in.speed_1_m_per_s_actual_value, in.speed_2_m_per_s_actual_value);
  double const angular_speed_deg_per_s_actual_value = _odometry.calcAngularSpeed(in.speed_1_m_per_s_actual_value, in.speed_2_m_per_s_actual_value);

  _differential_drive_regulator.updateWithActualValue(linear_x_m_per_s_actual_value, angular_speed_deg_per_s_actual_value);

  out->speed_1_m_per_s_target_value = _differential_drive_regulator.getSpeed_1_m_per_s();
  out->speed_2_m_per_s_target_value = _differential_drive_regulator.getSpeed_2_m_per_s();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/
