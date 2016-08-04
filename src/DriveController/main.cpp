/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static double const T_LOOP_UPDATE_s  = 0.2;   /* 200 ms */

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr    const & msg);
void actual_speed_2_callback(std_msgs::Float64::ConstPtr    const & msg);
void cmd_vel_callback       (geometry_msgs::Twist::ConstPtr const & msg);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller_node");

  ros::NodeHandle node_handle;

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

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void actual_speed_1_callback(std_msgs::Float64::ConstPtr const & msg)
{

}

void actual_speed_2_callback(std_msgs::Float64::ConstPtr const & msg)
{

}

void cmd_vel_callback(geometry_msgs::Twist::ConstPtr const & msg)
{

}
