/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "DifferentialDriveRegulator.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

DifferentialDriveRegulator::DifferentialDriveRegulator(double const kP_1, double const kI_1,
                                                         double const kP_2, double const kI_2,
                                                         double const dt_s)
: _linear_x_regulator               (kP_1, kI_1, dt_s, LINEAR_X_MIN_SPEED_m_per_s,    LINEAR_X_MAX_SPEED_m_per_s),
  _angular_z_regulator              (kP_2, kI_2, dt_s, ANGULAR_X_MIN_SPEED_deg_per_s, ANGULAR_X_MAX_SPEED_deg_per_s),
  _linear_x_m_per_s_target_value    (0.0),
  _angular_z_deg_per_s_target_value (0.0),
  _speed_1_m_per_s                  (0.0),
  _speed_2_m_per_s                  (0.0)
{

}

void DifferentialDriveRegulator::setLinearX(double const linear_x_m_per_s_target_value )
{
  _linear_x_m_per_s_target_value = linear_x_m_per_s_target_value;
}

void DifferentialDriveRegulator::setAngularZ(double const angular_z_deg_per_s_target_value)
{
  _angular_z_deg_per_s_target_value = angular_z_deg_per_s_target_value;
}

void DifferentialDriveRegulator::updateWithActualValue(double const linear_x_m_per_s_actual_value,
                                                        double const angular_z_deg_per_s_actual_value)
{
  double const linear_x_regulator_out   = _linear_x_regulator.calc  (_linear_x_m_per_s_target_value,    linear_x_m_per_s_actual_value   );
  double const angular_z_regulator_out  = _angular_z_regulator.calc (_angular_z_deg_per_s_target_value, angular_z_deg_per_s_actual_value);

  _speed_1_m_per_s = linear_x_regulator_out - angular_z_regulator_out;
  _speed_2_m_per_s = linear_x_regulator_out + angular_z_regulator_out;
}

double DifferentialDriveRegulator::getSpeed_1_m_per_s() const
{
  return _speed_1_m_per_s;
}

double DifferentialDriveRegulator::getSpeed_2_m_per_s() const
{
  return _speed_2_m_per_s;
}
