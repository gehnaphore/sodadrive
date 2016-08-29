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

#include "DriveController.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static DriveController::sIn drive_controller_in = {0.0, 0.0, 0.0, 0.0};

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr    const & msg);
void actual_speed_2_callback(std_msgs::Float64::ConstPtr    const & msg);
void cmd_vel_callback       (geometry_msgs::Twist::ConstPtr const & msg);
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

  DriveController drive_controller;

  /* Setup the publishers */

  ros::Publisher speed_1_publisher = node_handle.advertise<std_msgs::Float64>("/rpi/speed_1", 10);
  ros::Publisher speed_2_publisher = node_handle.advertise<std_msgs::Float64>("/rpi/speed_2", 10);

  /* Setup the subscribers */

  ros::Subscriber actual_speed_1_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/rpi/actual_speed_1", 10, actual_speed_1_callback);
  ros::Subscriber actual_speed_2_subscriber = node_handle.subscribe<std_msgs::Float64>   ("/rpi/actual_speed_2", 10, actual_speed_2_callback);

  ros::Subscriber cmd_vel_subscriber        = node_handle.subscribe<geometry_msgs::Twist>("/rpi/cmd_vel",         10, cmd_vel_callback);

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / DriveController::T_LOOP_UPDATE_s);

  while(ros::ok())
  {
    ros::spinOnce();

    DriveController::sOut drive_controller_out;

    drive_controller.run(drive_controller_in, &drive_controller_out);

    publishSpeed          (speed_1_publisher,
                           drive_controller_out.speed_1_m_per_s_target_value,
                           speed_2_publisher,
                           drive_controller_out.speed_2_m_per_s_target_value);

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr const & msg)
{
  drive_controller_in.speed_1_m_per_s_actual_value = msg->data;
}

void actual_speed_2_callback(std_msgs::Float64::ConstPtr const & msg)
{
  drive_controller_in.speed_2_m_per_s_actual_value = msg->data;
}

void cmd_vel_callback(geometry_msgs::Twist::ConstPtr const & msg)
{
  drive_controller_in.linear_x_m_per_s_target_value     = msg->linear.x;
  drive_controller_in.angular_z_deg_per_s_target_value  = msg->angular.z;
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
