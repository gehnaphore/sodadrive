/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

#include "MD49/MD49.h"
#include "MD49/EMG49.h"
#include "MD49/Serial.h"

#include "Regulator/Odometry.h"
#include "Regulator/PIDRegulator.h"
#include "Regulator/LoopPeriodAnalyser.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

/* I/O constants */

static std::string  const MD49_DEV_NODE     = "/dev/ttyUSB0";
static size_t       const MD49_BAUD_RATE    = 38400;

/* Geometrical constants */

static double       const WHEEL_DIAMETER_m  = 0.125;
static double       const WHEEL_DISTANCE_m  = 0.34;

/* Control constants */

static double       const T_LOOP_UPDATE_s   = 0.050;

static double       const kP_LINEAR         = 0.100;
static double       const kI_LINEAR         = 0.000;
static double       const kD_LINEAR         = 0.000;

static double       const kP_ANGULAR        = 0.000;
static double       const kI_ANGULAR        = 0.000;
static double       const kD_ANGULAR        = 0.000;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void cmdVelCallback         (geometry_msgs::Twist::ConstPtr const & msg,
                             double        * target_speed_linear_m_per_s,
                             double        * target_speed_angular_deg_per_s);

void getDeltaEncoderValues  (MD49    & md49,
                             int32_t & delta_encoder_1,
                             int32_t & delta_encoder_2);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_drive_controller_node");

  /* Instantiate all classes */

  ros::NodeHandle       node_handle;

  Serial                serial(MD49_DEV_NODE, MD49_BAUD_RATE, true);
  MD49                  md49  (serial);

  PIDRegulator          linear_x_regulator(kP_LINEAR,
                                           kI_LINEAR,
                                           kD_LINEAR,
                                           T_LOOP_UPDATE_s,
                                           -1.0,
                                           1.0);

  PIDRegulator          angular_z_regulator(kP_ANGULAR,
                                            kI_ANGULAR,
                                            kD_ANGULAR,
                                            T_LOOP_UPDATE_s,
                                            -1.0,
                                            1.0);

  double                target_speed_linear_m_per_s     = 0.0,
                        target_speed_angular_deg_per_s  = 0.0,
                        speed_1                         = 0.0,
                        speed_2                         = 0.0;

  /* Setup subscribers */

  ros::Subscriber cmd_vel_subscriber  = node_handle.subscribe<geometry_msgs::Twist>("/rpi/cmd_vel",
                                                                                    10,
                                                                                    boost::bind(&cmdVelCallback, _1, &target_speed_linear_m_per_s, &target_speed_angular_deg_per_s));

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / T_LOOP_UPDATE_s);

  LoopPeriodAnalyser  loop_period_analyser;

  while(ros::ok())
  {
    ros::spinOnce();

    int32_t delta_encoder_1 = 0,
            delta_encoder_2 = 0;

    getDeltaEncoderValues     (md49,
                               delta_encoder_1,
                               delta_encoder_2);

    loop_period_analyser.update();

    double const loop_durations_s               = loop_period_analyser.getLoopPeriodInSec();
    double const actual_speed_1_m_per_s         = EMG49::calcDistanceTraveled_m(delta_encoder_1, WHEEL_DIAMETER_m) / loop_durations_s;
    double const actual_speed_2_m_per_s         = EMG49::calcDistanceTraveled_m(delta_encoder_2, WHEEL_DIAMETER_m) / loop_durations_s;

    double const actual_linear_speed_m_per_s    = Odometry::calcLinearSpeed_m_per_s(WHEEL_DISTANCE_m, actual_speed_1_m_per_s, actual_speed_2_m_per_s);
    double const actual_angular_speed_deg_per_s = 0.0; /* TODO: Retrieve from Pozxy */

    double const linear_x_regulator_out         = linear_x_regulator.calc  (target_speed_linear_m_per_s,    actual_linear_speed_m_per_s   );
    double const angular_z_regulator_out        = angular_z_regulator.calc (target_speed_angular_deg_per_s, actual_angular_speed_deg_per_s);

    speed_1                                     = linear_x_regulator_out - angular_z_regulator_out;
    speed_2                                     = linear_x_regulator_out + angular_z_regulator_out;

    speed_1 = std::max(speed_1, -1.0);
    speed_1 = std::min(speed_1,  1.0);

    speed_2 = std::max(speed_2, -1.0);
    speed_2 = std::min(speed_2,  1.0);

    md49.setSpeed1(static_cast<int8_t>(speed_1 * 127.0));
    md49.setSpeed2(static_cast<int8_t>(speed_2 * 127.0));

    ROS_INFO("dt = %f, v_1 = %f, v_2 = %f, v_l = %f, v_a = %f, l_out = %f, a_out = %f, s_1 = %f, s_2 = %f",
             loop_durations_s,
             actual_speed_1_m_per_s,
             actual_speed_2_m_per_s,
             actual_linear_speed_m_per_s,
             actual_angular_speed_deg_per_s,
             linear_x_regulator_out,
             angular_z_regulator_out,
             speed_1,
             speed_2);

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

void cmdVelCallback(geometry_msgs::Twist::ConstPtr const & msg,
                    double        * target_speed_linear_m_per_s,
                    double        * target_speed_angular_deg_per_s)
{
  *target_speed_linear_m_per_s    = msg->linear.x;
  *target_speed_angular_deg_per_s = msg->angular.z;
}

void getDeltaEncoderValues(MD49    & md49,
                           int32_t & delta_encoder_1,
                           int32_t & delta_encoder_2)
{
  static int32_t prev_encoder_1    = 0;
  static int32_t prev_encoder_2    = 0;

         int32_t current_encoder_1 = 0,
                 current_encoder_2 = 0;

  md49.getEncoders(current_encoder_1, current_encoder_2);

  delta_encoder_1 = current_encoder_1 - prev_encoder_1;
  delta_encoder_2 = current_encoder_2 - prev_encoder_2;

  prev_encoder_1  = current_encoder_1;
  prev_encoder_2  = current_encoder_2;
}
