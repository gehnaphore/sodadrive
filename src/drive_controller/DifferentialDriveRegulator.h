/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_
#define RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "../MD49/PIRegulator.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DifferentialDriveRegulator
{

public:

  DifferentialDriveRegulator(double const kP_1, double const kI_1,
                             double const kP_2, double const kI_2,
                             double const dt_s);

  void    setLinearX            (double const linear_x_m_per_s_target_value   );
  void    setAngularZ           (double const angular_deg_m_per_s_target_value);

  void    updateWithActualValue (double const linear_x_m_per_s_actual_value,
                                 double const angular_z_deg_per_s_actual_value);

  double  getSpeed_1_m_per_s    (                                             ) const;
  double  getSpeed_2_m_per_s    (                                             ) const;

private:

  PIRegulator _linear_x_regulator,
              _angular_z_regulator;

  double      _linear_x_m_per_s_target_value,
              _angular_z_deg_per_s_target_value;

  double      _speed_1_m_per_s,
              _speed_2_m_per_s;

  static double const LINEAR_X_MIN_SPEED_m_per_s    =  -1.0;
  static double const LINEAR_X_MAX_SPEED_m_per_s    =   1.0;
  static double const ANGULAR_X_MIN_SPEED_deg_per_s = -25.0;
  static double const ANGULAR_X_MAX_SPEED_deg_per_s =  25.0;

};

#endif /* RPI_SRC_DRIVE_CONTROLLER_DIFFERENTIALDRIVEREGULATOR_H_ */
