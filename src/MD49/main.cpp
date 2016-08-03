/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>

#include <ros/ros.h>

#include "MD49.h"
#include "Serial.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static size_t const MD49_BAUD_RATE        = 38400;
static double const RD03_WHEEL_DIAMETER_m = 0.125;

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "md49_node");

  /* Instantiate all classes */

  ros::NodeHandle node_handle;
  Serial          serial      ("/dev/ttyUSB0", MD49_BAUD_RATE);
  MD49            md49        (serial);

  /* Loop forever */

  ros::spin();

  return EXIT_SUCCESS;
}
