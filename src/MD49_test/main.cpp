/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void handleSetSpeed1   (ros::Publisher &speed_1_publisher);
void handleSetSpeed2   (ros::Publisher &speed_2_publisher);
void handleExit        ();
void handleInvalidValue();

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MD49_test");

  ros::NodeHandle node_handle;

  /* Setup the publishers */

  ros::Publisher speed_1_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_1", 10);
  ros::Publisher speed_2_publisher = node_handle.advertise<std_msgs::Float64>("/md49/speed_2", 10);

  /* Provide a crude menu for selecting which service one wants to invoke */

  char cmd = 0;

  do
  {
    std::cout                               << std::endl;
    std::cout << "[1] set speed of motor 1" << std::endl;
    std::cout << "[2] set speed of motor 2" << std::endl;
    std::cout << "[q]uit"                   << std::endl;
    std::cout << ">>"; std::cin >> cmd;

    switch(cmd)
    {
    case '1': handleSetSpeed1   (speed_1_publisher);  break;
    case '2': handleSetSpeed2   (speed_2_publisher);  break;
    case 'q': handleExit        ();                   break;
    default:  handleInvalidValue();                   break;
    }
  } while(cmd != 'q');

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * OUR FUNCTIONS
 **************************************************************************************/

void handleSetSpeed1(ros::Publisher &speed_1_publisher)
{
  double speed_1_m_per_s;

  std::cout << "Speed 1 (m / s) >> "; std::cin >> speed_1_m_per_s;

  std_msgs::Float64 msg; msg.data = speed_1_m_per_s;

  speed_1_publisher.publish(msg);
}

void handleSetSpeed2(ros::Publisher &speed_2_publisher)
{
  double speed_2_m_per_s;

  std::cout << "Speed 2 (m / s) >> "; std::cin >> speed_2_m_per_s;

  std_msgs::Float64 msg; msg.data = speed_2_m_per_s;

  speed_2_publisher.publish(msg);
}

void handleExit()
{
  std::cout << "Exiting function MD49_test" << std::endl;
}

void handleInvalidValue()
{
  std::cout << "Invalid input value" << std::endl;
}
