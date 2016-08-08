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
  double linear_x_m_per_s_target_value;
  double angular_z_deg_per_s_target_value;
  double actual_speed_1_m_per_s;
  double actual_speed_2_m_per_s;
} sDifferentialDriveControllerInputData;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const T_LOOP_UPDATE_s  = 0.2;   /* 200 ms */
static double const WHEEL_DISTANCE_m = 0.5;   /* 0.5 m */

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static sDifferentialDriveControllerInputData drive_controller_input_data = { 0.0, 0.0, 0.0, 0.0 };

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr    const & msg);
void actual_speed_2_callback(std_msgs::Float64::ConstPtr    const & msg);
void cmd_vel_callback       (geometry_msgs::Twist::ConstPtr const & msg);
void setTargetSpeed         (DifferentialDriveController          & drive_controller);
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

  DifferentialDriveController drive_controller(1.0, 1.0, 1.0, 1.0, T_LOOP_UPDATE_s);
  Odometry                    odometry(WHEEL_DISTANCE_m);

  /* Setup the publishers */

  ros::Publisher speed_1_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_1", 10);
  ros::Publisher speed_2_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_2", 10);

  /* Setup the subscribers */

  ros::Subscriber actual_speed_1_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/md49/actual_speed_1", 10, actual_speed_1_callback);
  ros::Subscriber actual_speed_2_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/md49/actual_speed_2", 10, actual_speed_2_callback);

  ros::Subscriber cmd_vel_subscriber        = node_handle.subscribe<geometry_msgs::Twist>("/rpi/cmd_vel",         10, cmd_vel_callback       );

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / T_LOOP_UPDATE_s);

  while(ros::ok())
  {
    ros::spinOnce();

    setTargetSpeed        (drive_controller);

    double linear_x_m_per_s_actual_value        = 0.0,
           angular_speed_deg_per_s_actual_value = 0.0;

    getActualSpeed        (odometry,
                           linear_x_m_per_s_actual_value,
                           angular_speed_deg_per_s_actual_value);

    drive_controller.updateWithActualValue
                          (linear_x_m_per_s_actual_value,
                           angular_speed_deg_per_s_actual_value);

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
  drive_controller_input_data.actual_speed_1_m_per_s = msg->data;
}

void actual_speed_2_callback(std_msgs::Float64::ConstPtr const & msg)
{
  drive_controller_input_data.actual_speed_2_m_per_s = msg->data;
}

void cmd_vel_callback(geometry_msgs::Twist::ConstPtr const & msg)
{
  drive_controller_input_data.linear_x_m_per_s_target_value     = msg->linear.x;
  drive_controller_input_data.angular_z_deg_per_s_target_value  = msg->angular.z;
}

void setTargetSpeed(DifferentialDriveController &drive_controller)
{
  drive_controller.setLinearX (drive_controller_input_data.linear_x_m_per_s_target_value);
  drive_controller.setAngularZ(drive_controller_input_data.angular_z_deg_per_s_target_value);
}

void getActualSpeed(Odometry &odometry, double &linear_x_m_per_s_actual_value, double &angular_speed_deg_per_s_actual_value)
{
  linear_x_m_per_s_actual_value        = odometry.calcLinearSpeed (drive_controller_input_data.actual_speed_1_m_per_s, drive_controller_input_data.actual_speed_2_m_per_s);
  angular_speed_deg_per_s_actual_value = odometry.calcAngularSpeed(drive_controller_input_data.actual_speed_1_m_per_s, drive_controller_input_data.actual_speed_2_m_per_s);
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
  speed_1_publisher.publish(speed_2_msg);
}
