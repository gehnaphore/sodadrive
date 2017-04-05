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
#include <std_msgs/Float64.h>

#include "MD49.h"
#include "Serial.h"
#include "WheelController.h"
#include "Odometry.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/
 
static size_t const MD49_BAUD_RATE   = 38400;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

WheelController::sIn wheel_controller_in = {0.0, 0.0};
bool sReverseWheels = true;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void speed_1_callback      (std_msgs::Float64::ConstPtr const & msg);
void speed_2_callback      (std_msgs::Float64::ConstPtr const & msg);
void publishActualSpeed    (ros::Publisher&      actual_speed_1_publisher,
                            double        const  actual_speed_1_m_per_s,
                            ros::Publisher&      actual_speed_2_publisher,
                            double        const  actual_speed_2_m_per_s);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_controller_node");

  /* Instantiate all classes */

  ros::NodeHandle       node_handle;

  Serial                serial            ("/dev/sodabot/md49", MD49_BAUD_RATE, true);
  MD49                  md49              (serial);
  WheelController       wheel_controller  (&md49);
  Odometry_calc         odom              (sReverseWheels);

  /* Setup subscribers */

  ros::Subscriber speed_1_subscriber       = node_handle.subscribe<std_msgs::Float64>("speed_1",        10, speed_1_callback);
  ros::Subscriber speed_2_subscriber       = node_handle.subscribe<std_msgs::Float64>("speed_2",        10, speed_2_callback);

  ros::Publisher  actual_speed_1_publisher = node_handle.advertise<std_msgs::Float64>("actual_speed_1", 10);
  ros::Publisher  actual_speed_2_publisher = node_handle.advertise<std_msgs::Float64>("actual_speed_2", 10);

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / WheelController::T_LOOP_UPDATE_s);

  while(ros::ok())
  {
    ros::spinOnce();

    WheelController::sOut wheel_controller_out;

    wheel_controller.run(wheel_controller_in, &wheel_controller_out);

    publishActualSpeed       (actual_speed_1_publisher,
                              wheel_controller_out.speed_1_m_per_s_actual_value,
                              actual_speed_2_publisher,
                              wheel_controller_out.speed_2_m_per_s_actual_value);

    int32_t left,right;
    wheel_controller.getLastEncoderValues(left,right);
    if (sReverseWheels) {
      odom.updateLeftEncoder(-right);
      odom.updateRightEncoder(-left);
    } else {
      odom.updateLeftEncoder(left);
      odom.updateRightEncoder(right);
    }
    odom.publishOdometry();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

void speed_1_callback(std_msgs::Float64::ConstPtr const & msg)
{
  if (sReverseWheels) {
    wheel_controller_in.speed_2_m_per_s_target_value  = -msg->data;
  } else {
    wheel_controller_in.speed_1_m_per_s_target_value  = msg->data;
  }
}

void speed_2_callback(std_msgs::Float64::ConstPtr const & msg)
{
  if (sReverseWheels) {
    wheel_controller_in.speed_1_m_per_s_target_value  = -msg->data;
  } else {
    wheel_controller_in.speed_2_m_per_s_target_value  = msg->data;
  }
}

void publishActualSpeed(ros::Publisher &actual_speed_1_publisher, double const actual_speed_1_m_per_s,
                        ros::Publisher &actual_speed_2_publisher, double const actual_speed_2_m_per_s)
{
  std_msgs::Float64 actual_speed_1_msg; actual_speed_1_msg.data = sReverseWheels ? -actual_speed_2_m_per_s : actual_speed_1_m_per_s;
  std_msgs::Float64 actual_speed_2_msg; actual_speed_2_msg.data = sReverseWheels ? -actual_speed_1_m_per_s : actual_speed_2_m_per_s;

  actual_speed_1_publisher.publish(actual_speed_1_msg);
  actual_speed_2_publisher.publish(actual_speed_2_msg);
}
