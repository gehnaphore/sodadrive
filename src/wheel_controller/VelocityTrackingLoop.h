/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_WHEEL_CONTROLLER_VELOCITYTRACKINGLOOP_H_
#define RPI_SRC_WHEEL_CONTROLLER_VELOCITYTRACKINGLOOP_H_

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class VelocityTrackingLoop
{

public:

  VelocityTrackingLoop(double const kP, double const kI);

  void  update      (double const position_measured, double const dt_s);
  float getVelocity (                                                 ) const;

private:

  double  _kP, _kI;

  double  _velocity_estimated,
          _velocity_integrated,
          _position_estimated;

};



#endif /* RPI_SRC_WHEEL_CONTROLLER_VELOCITYTRACKINGLOOP_H_ */
