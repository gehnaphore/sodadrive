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

  Odometry(double const wheel_distance_m);

  double calcLinearSpeed  (double const speed_left_m_per_s, double const speed_right_m_per_s);
  double calcAngularSpeed (double const speed_left_m_per_s, double const speed_right_m_per_s);

private:

  double _wheel_distance_m;

};

#endif /* RPI_SRC_DRIVE_CONTROLLER_ODOMETRY_H_ */
