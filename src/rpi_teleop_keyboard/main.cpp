/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <boost/thread.hpp>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

double const LINEAR_SPEED_INCREMENT_m_per_s       = 0.02;
double const ANGULAR_SPEED_INCREMENT_deg_per_s    = 1;

double const MIN_LINEAR_SPEED_m_per_s             = -1.0;
double const MAX_LINEAR_SPEED_m_per_s             = 1.0;
double const MIN_ANGULAR_SPEED_m_per_s            = -20.0;
double const MAX_ANGULAR_SPEED_m_per_s            = 20.0;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

char getch();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rpi_teleop_keyboard_node");

  ros::NodeHandle node_handle;

  /* Setup the publishers */

  ros::Publisher cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/rpi/cmd_vel",  10);

  /* Select current linear and angular speed via keyboard entries */

  std::cout << "[w] -> increase linear speed" << std::endl;
  std::cout << "[s] -> decrease linear speed" << std::endl;
  std::cout << "[a] -> decrease angular speed" << std::endl;
  std::cout << "[d] -> increase angular speed" << std::endl;
  std::cout << "[q]uit" << std::endl;

  double current_linear_speed_m_per_s   = 0.0,
         current_angular_speed_m_per_s  = 0.0;

  char cmd = 0;
  do
  {
    cmd = getch();

    switch(cmd)
    {
    case 'w':
    {
      current_linear_speed_m_per_s += LINEAR_SPEED_INCREMENT_m_per_s;
    }
    break;
    case 's':
    {
      current_linear_speed_m_per_s -= LINEAR_SPEED_INCREMENT_m_per_s;
    }
    break;
    case 'a':
    {
      current_angular_speed_m_per_s -= ANGULAR_SPEED_INCREMENT_deg_per_s;
    }
    break;
    case 'd':
    {
      current_angular_speed_m_per_s += ANGULAR_SPEED_INCREMENT_deg_per_s;
    }
    break;
    case 'q':
    {
      current_linear_speed_m_per_s  = 0.0;
      current_angular_speed_m_per_s = 0.0;
    }
    break;
    default:
    {
      current_linear_speed_m_per_s  = 0.0;
      current_angular_speed_m_per_s = 0.0;
    }
    break;
    }

    /* Limit linear and angular speed */

    current_linear_speed_m_per_s  = std::max<double>(current_linear_speed_m_per_s,  MIN_LINEAR_SPEED_m_per_s);
    current_linear_speed_m_per_s  = std::min<double>(current_linear_speed_m_per_s,  MAX_LINEAR_SPEED_m_per_s);

    current_angular_speed_m_per_s = std::max<double>(current_angular_speed_m_per_s, MIN_ANGULAR_SPEED_m_per_s);
    current_angular_speed_m_per_s = std::min<double>(current_angular_speed_m_per_s, MAX_ANGULAR_SPEED_m_per_s);


    /* Publish the message */

    geometry_msgs::Twist msg;

    msg.linear.x  = current_linear_speed_m_per_s;
    msg.angular.z = current_angular_speed_m_per_s;

    cmd_vel_publisher.publish(msg);

  } while(cmd != 'q');

  /* Ensure that the last message is sent too */

  boost::this_thread::sleep_for(boost::chrono::seconds(2));

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

/* http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed */
char getch()
{
  char buf = 0;
  struct termios old = { 0 };
  if (tcgetattr(0, &old) < 0)             perror("tcsetattr()");

  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;

  if (tcsetattr(0, TCSANOW, &old) < 0)    perror("tcsetattr ICANON");

  if (read(0, &buf, 1) < 0)               perror("read()");

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;

  if (tcsetattr(0, TCSADRAIN, &old) < 0)  perror("tcsetattr ~ICANON");
  return (buf);
}
