/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_DRIVE_CONTROLLER_ODOMETRY_H_
#define RPI_SRC_DRIVE_CONTROLLER_ODOMETRY_H_

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Odometry
{

public:

  static double calcLinearSpeed_m_per_s(double const wheel_distance_m, double const speed_left_m_per_s, double const speed_right_m_per_s);

private:

  Odometry() { }
  Odometry(Odometry const &other) { }

};

#endif /* RPI_SRC_DRIVE_CONTROLLER_ODOMETRY_H_ */
