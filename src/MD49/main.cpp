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
#include "EMG49.h"
#include "Serial.h"
#include "MD49Regulator.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static size_t const MD49_BAUD_RATE   = 38400;
static double const WHEEL_DIAMETER_m = 0.125;
static double const T_LOOP_UPDATE_s  = 0.1;    /* 100 ms */

/**************************************************************************************
 * PROTOTYPES
 **************************************************************************************/

void speed_1_callback      (std_msgs::Float64::ConstPtr const & msg, MD49Regulator *md49_regulator );
void speed_2_callback      (std_msgs::Float64::ConstPtr const & msg, MD49Regulator *md49_regulator );

void getDeltaEncoderValues    (MD49          &      md49,
                               int32_t       &      delta_encoder_1,
                               int32_t       &      delta_encoder_2);

void getActualSpeed_m_per_s   (int32_t       const  delta_encoder_1,
                               int32_t       const  delta_encoder_2,
                               double        &      actual_speed_1_m_per_s,
                               double        &      actual_speed_2_m_per_s);

void getSpeedFromRegulator    (MD49Regulator *      md49_regulator,
                               int8_t        &      speed_1,
                               int8_t        &      speed_2);

void setSpeedAtMotorController(MD49          &      md49,
                               int8_t        const  speed_1,
                               int8_t        const  speed_2);

void publishActualSpeed       (ros::Publisher&      actual_speed_1_publisher,
                               double        const  actual_speed_1_m_per_s,
                               ros::Publisher&      actual_speed_2_publisher,
                               double        const  actual_speed_2_m_per_s);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "md49_node");

  /* Instantiate all classes */

  ros::NodeHandle node_handle;
  Serial          serial        ("/dev/ttyUSB0", MD49_BAUD_RATE);
  MD49            md49          (serial);
  MD49Regulator   md49_regulator(0.0, 200.0, 0.0, 200.0, T_LOOP_UPDATE_s);

  /* Setup subscribers */

  ros::Subscriber speed_1_subscriber       = node_handle.subscribe<std_msgs::Float64>("/md49/speed_1",        10, boost::bind(&speed_1_callback, _1, &md49_regulator));
  ros::Subscriber speed_2_subscriber       = node_handle.subscribe<std_msgs::Float64>("/md49/speed_2",        10, boost::bind(&speed_2_callback, _1, &md49_regulator));

  ros::Publisher  actual_speed_1_publisher = node_handle.advertise<std_msgs::Float64>("/md49/actual_speed_1", 10);
  ros::Publisher  actual_speed_2_publisher = node_handle.advertise<std_msgs::Float64>("/md49/actual_speed_2", 10);

  /* Run the control loop */

  ros::Rate loop_rate(1.0 / T_LOOP_UPDATE_s);

  while(ros::ok())
  {
    ros::spinOnce();

    int32_t delta_encoder_1 = 0,
            delta_encoder_2 = 0;

    getDeltaEncoderValues    (md49,
                              delta_encoder_1,
                              delta_encoder_2);

    double actual_speed_1_m_per_s = 0.0,
           actual_speed_2_m_per_s = 0.0;

    getActualSpeed_m_per_s   (delta_encoder_1,
                              delta_encoder_2,
                              actual_speed_1_m_per_s,
                              actual_speed_2_m_per_s);
    md49_regulator.updateWithActualValue(actual_speed_1_m_per_s, actual_speed_2_m_per_s);

    int8_t speed_1 = 0,
           speed_2 = 0;

    getSpeedFromRegulator    (&md49_regulator,
                              speed_1,
                              speed_2);

    setSpeedAtMotorController(md49,
                              speed_1,
                              speed_2);

    publishActualSpeed       (actual_speed_1_publisher,
                              actual_speed_1_m_per_s,
                              actual_speed_2_publisher,
                              actual_speed_2_m_per_s);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

void speed_1_callback(std_msgs::Float64::ConstPtr const & msg, MD49Regulator *md49_regulator)
{
  md49_regulator->setSpeed1TargetValue(msg->data);
}

void speed_2_callback(std_msgs::Float64::ConstPtr const & msg, MD49Regulator *md49_regulator)
{
  md49_regulator->setSpeed2TargetValue(msg->data);
}

void getDeltaEncoderValues(MD49 &md49, int32_t &delta_encoder_1, int32_t &delta_encoder_2)
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

void getActualSpeed_m_per_s(int32_t const delta_encoder_1, int32_t const delta_encoder_2,
                            double &actual_speed_1_m_per_s, double &actual_speed_2_m_per_s)
{
  actual_speed_1_m_per_s = EMG49::calcDistanceTraveled_m(delta_encoder_1, WHEEL_DIAMETER_m) / T_LOOP_UPDATE_s;
  actual_speed_2_m_per_s = EMG49::calcDistanceTraveled_m(delta_encoder_2, WHEEL_DIAMETER_m) / T_LOOP_UPDATE_s;
}

void getSpeedFromRegulator(MD49Regulator *md49_regulator, int8_t &speed_1, int8_t &speed_2)
{
  speed_1 = md49_regulator->getSpeed1();
  speed_2 = md49_regulator->getSpeed2();
}

void setSpeedAtMotorController(MD49 &md49, int8_t const speed_1, int8_t const speed_2)
{
  md49.setSpeed1(speed_1);
  md49.setSpeed2(speed_2);
}

void publishActualSpeed(ros::Publisher &actual_speed_1_publisher, double const actual_speed_1_m_per_s,
                        ros::Publisher &actual_speed_2_publisher, double const actual_speed_2_m_per_s)
{
  std_msgs::Float64 actual_speed_1_msg; actual_speed_1_msg.data = actual_speed_1_m_per_s;
  std_msgs::Float64 actual_speed_2_msg; actual_speed_2_msg.data = actual_speed_2_m_per_s;

  actual_speed_1_publisher.publish(actual_speed_1_msg);
  actual_speed_2_publisher.publish(actual_speed_2_msg);
}
