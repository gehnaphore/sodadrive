/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_DRIVE_CONTROLLER_DRIVECONTROLLER_H_
#define RPI_SRC_DRIVE_CONTROLLER_DRIVECONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "DifferentialDriveRegulator.h"
#include "Odometry.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DriveController
{

public:

  static double const T_LOOP_UPDATE_s  = 0.2;    /* 200 ms */

  DriveController();

  typedef struct
  {
    float speed_1_m_per_s_actual_value;
    float speed_2_m_per_s_actual_value;
    float linear_x_m_per_s_target_value;
    float angular_z_rad_per_s_target_value;
  } sIn;

  typedef struct
  {
    float speed_1_m_per_s_target_value;
    float speed_2_m_per_s_target_value;
  } sOut;

  void run(sIn const &in, sOut *out);

private:

  Odometry                    _odometry;
  DifferentialDriveRegulator  _differential_drive_regulator;


};

#endif /* RPI_SRC_DRIVE_CONTROLLER_DRIVECONTROLLER_H_ */
