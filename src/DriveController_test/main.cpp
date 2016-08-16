/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void handleSetLinearSpeed   (ros::Publisher &cmd_vel_publisher, double &current_linear_speed_m_per_s, double &current_angular_speed_m_per_s);
void handleSetAngularSpeed  (ros::Publisher &cmd_vel_publisher, double &current_linear_speed_m_per_s, double &current_angular_speed_m_per_s);
void handleExit             ();
void handleInvalidValue     ();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller_test");

  ros::NodeHandle node_handle;

  /* Setup the publishers */

  ros::Publisher cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/rpi/cmd_vel",  10);

  /* Provide a crude menu for selecting which service one wants to invoke */

  char cmd = 0;

  double current_linear_speed_m_per_s   = 0.0,
         current_angular_speed_m_per_s  = 0.0;

  do
  {
    std::cout                               << std::endl;
    std::cout << "[1] set linear speed"     << std::endl;
    std::cout << "[2] set angular speed"    << std::endl;
    std::cout << "[q]uit"                   << std::endl;
    std::cout << ">>"; std::cin >> cmd;

    switch(cmd)
    {
    case '1': handleSetLinearSpeed   (cmd_vel_publisher, current_linear_speed_m_per_s, current_angular_speed_m_per_s);  break;
    case '2': handleSetAngularSpeed  (cmd_vel_publisher, current_linear_speed_m_per_s, current_angular_speed_m_per_s);  break;
    case 'q': handleExit             ();                                                                                break;
    default:  handleInvalidValue     ();                                                                                break;
    }
  } while(cmd != 'q');

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void handleSetLinearSpeed(ros::Publisher &cmd_vel_publisher, double &current_linear_speed_m_per_s, double &current_angular_speed_m_per_s)
{
  std::cout << "Linear Speed (m / s) >> "; std::cin >> current_linear_speed_m_per_s;

  geometry_msgs::Twist msg;
  msg.linear.x  = current_linear_speed_m_per_s;
  msg.angular.z = current_angular_speed_m_per_s;

  std::cout << msg << std::endl;

  cmd_vel_publisher.publish(msg);
}

void handleSetAngularSpeed(ros::Publisher &cmd_vel_publisher, double &current_linear_speed_m_per_s, double &current_angular_speed_m_per_s)
{
  std::cout << "Angular Speed (deg / s) >> "; std::cin >> current_angular_speed_m_per_s;

  geometry_msgs::Twist msg;
  msg.linear.x  = current_linear_speed_m_per_s;
  msg.angular.z = current_angular_speed_m_per_s;

  std::cout << msg << std::endl;

  cmd_vel_publisher.publish(msg);
}

void handleExit()
{
  std::cout << "Exiting node drive_controller_test" << std::endl;
}

void handleInvalidValue()
{
  std::cout << "Invalid input value" << std::endl;
}
