/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_
#define RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "../wheel_controller/PIDRegulator.h"
#include <math.h>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DifferentialDriveRegulator
{

public:

  DifferentialDriveRegulator(double const d_m,
                             double const kP_1, double const kI_1, double const kD_1,
                             double const kP_2, double const kI_2, double const kD_2,
                             double const dt_s);

  void    setLinearX            (double const linear_x_m_per_s_target_value   );
  void    setAngularZ           (double const angular_rad_m_per_s_target_value);

  void    updateWithActualValue (double const linear_x_m_per_s_actual_value,
                                 double const angular_z_rad_per_s_actual_value);

  double  getSpeed_1_m_per_s    (                                             ) const;
  double  getSpeed_2_m_per_s    (                                             ) const;

private:

  PIDRegulator  _linear_x_regulator,
                _angular_z_regulator;

  double        _linear_x_m_per_s_target_value,
                _angular_z_rad_per_s_target_value;

  double        _speed_1_m_per_s,
                _speed_2_m_per_s;

  double        _dm;

  static double const LINEAR_X_MIN_SPEED_m_per_s    =  -1.0;
  static double const LINEAR_X_MAX_SPEED_m_per_s    =   1.0;
  static double const ANGULAR_X_MIN_SPEED_rad_per_s = -M_PI;
  static double const ANGULAR_X_MAX_SPEED_rad_per_s =  M_PI;

};

#endif /* RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_ */
