/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include "Odometry.h"
#include "DifferentialDriveController.h"

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef struct
{
  double speed_1_m_per_s;
  double speed_2_m_per_s;
} ActualSpeed;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const T_LOOP_UPDATE_s  = 0.10;  /* 100  ms */
static double const WHEEL_DISTANCE_m = 0.34;  /* 0.34 m  */

static double const kP_LINEAR        = 0.50;
static double const kI_LINEAR        = 1.00;
static double const kP_ANGULAR       = 0.00;
static double const kI_ANGULAR       = 0.01;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static ActualSpeed actual_speed = { 0.0, 0.0 };

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr    const & msg);
void actual_speed_2_callback(std_msgs::Float64::ConstPtr    const & msg);
void cmd_vel_callback       (geometry_msgs::Twist::ConstPtr const & msg,
                             DifferentialDriveController          * drive_controller);
void getActualSpeed         (Odometry                             & odometry,
                             double                               & linear_x_m_per_s_actual_value,
                             double                               & angular_speed_deg_per_s_actual_value);
void getSpeedFromRegulator  (DifferentialDriveController    const & drive_controller,
                             double                               & speed_1_m_per_s,
                             double                               & speed_2_m_per_s);
void publishSpeed           (ros::Publisher                       & speed_1_publisher,
                             double                         const speed_1_m_per_s,
                             ros::Publisher                       & speed_2_publisher,
                             double                         const speed_2_m_per_s);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller_node");

  ros::NodeHandle node_handle;

  /* Instantiate the classes */

  DifferentialDriveController drive_controller(kP_LINEAR,
                                               kI_LINEAR,
                                               kP_ANGULAR,
                                               kI_ANGULAR,
                                               T_LOOP_UPDATE_s);
  Odometry                    odometry        (WHEEL_DISTANCE_m);

  /* Setup the publishers */

  ros::Publisher speed_1_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_1", 10);
  ros::Publisher speed_2_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_2", 10);

  /* Setup the subscribers */

  ros::Subscriber actual_speed_1_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/md49/actual_speed_1", 10, actual_speed_1_callback);
  ros::Subscriber actual_speed_2_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/md49/actual_speed_2", 10, actual_speed_2_callback);

  ros::Subscriber cmd_vel_subscriber        = node_handle.subscribe<geometry_msgs::Twist>("/rpi/cmd_vel",         10, boost::bind(&cmd_vel_callback, _1, &drive_controller));

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / T_LOOP_UPDATE_s);

  while(ros::ok())
  {
    ros::spinOnce();

    double linear_x_m_per_s_actual_value        = 0.0,
           angular_speed_deg_per_s_actual_value = 0.0;

    getActualSpeed        (odometry,
                           linear_x_m_per_s_actual_value,
                           angular_speed_deg_per_s_actual_value);

    drive_controller.updateWithActualValue
                          (linear_x_m_per_s_actual_value,
                           angular_speed_deg_per_s_actual_value);

    std::cout << "LINEAR = " << linear_x_m_per_s_actual_value << " | "
              << "ANGULAR = " << angular_speed_deg_per_s_actual_value << std::endl; 

    double speed_1_m_per_s = 0.0,
           speed_2_m_per_s = 0.0;

    getSpeedFromRegulator (drive_controller,
                           speed_1_m_per_s,
                           speed_2_m_per_s);

    /* TODO limit acceleration */

    publishSpeed          (speed_1_publisher,
                           speed_1_m_per_s,
                           speed_2_publisher,
                           speed_2_m_per_s);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr const & msg)
{
  actual_speed.speed_1_m_per_s = msg->data;
}

void actual_speed_2_callback(std_msgs::Float64::ConstPtr const & msg)
{
  actual_speed.speed_2_m_per_s = msg->data;
}

void cmd_vel_callback(geometry_msgs::Twist::ConstPtr const & msg, DifferentialDriveController *drive_controller)
{
  drive_controller->setLinearX  (msg->linear.x  );
  drive_controller->setAngularZ (msg->angular.z );
}

void getActualSpeed(Odometry &odometry, double &linear_x_m_per_s_actual_value, double &angular_speed_deg_per_s_actual_value)
{
  linear_x_m_per_s_actual_value        = odometry.calcLinearSpeed (actual_speed.speed_1_m_per_s, actual_speed.speed_2_m_per_s);
  angular_speed_deg_per_s_actual_value = odometry.calcAngularSpeed(actual_speed.speed_1_m_per_s, actual_speed.speed_2_m_per_s);
}

void getSpeedFromRegulator(DifferentialDriveController const &drive_controller, double &speed_1_m_per_s, double &speed_2_m_per_s)
{
  speed_1_m_per_s = drive_controller.getSpeed_1_m_per_s();
  speed_2_m_per_s = drive_controller.getSpeed_2_m_per_s();
}

void publishSpeed(ros::Publisher &speed_1_publisher, double const speed_1_m_per_s, ros::Publisher &speed_2_publisher, double const speed_2_m_per_s)
{
  std_msgs::Float64 speed_1_msg;
  speed_1_msg.data = speed_1_m_per_s;
  speed_1_publisher.publish(speed_1_msg);

  std_msgs::Float64 speed_2_msg;
  speed_2_msg.data = speed_2_m_per_s;
  speed_2_publisher.publish(speed_2_msg);
}
