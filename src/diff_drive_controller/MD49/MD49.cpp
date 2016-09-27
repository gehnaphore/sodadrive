/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "MD49.h"

#include "MD49MessageCreator.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

MD49::MD49(Serial &serial)
: _serial(serial)
{
  enableTimeout   (       );
  disableRegulator(       );
  setMode         (MODE_4 );
  setSpeed1       (0      );
  setSpeed2       (0      );
  resetEncoders   (       );
}

void MD49::setSpeed1(int8_t const speed_1)
{
  _serial.write(MD49MessageCreator::createSetSpeed1Message(static_cast<uint8_t>(speed_1)));
}

void MD49::setSpeed2(int8_t const speed_2)
{
  _serial.write(MD49MessageCreator::createSetSpeed2Message(static_cast<uint8_t>(speed_2)));
}

void MD49::getEncoders(int32_t &encoder_1, int32_t &encoder_2)
{
  _serial.write(MD49MessageCreator::createGetEncodersMessage());

  MD49Message const encoder_msg = _serial.read(8);

  encoder_1 = static_cast<int32_t>(
              (static_cast<uint32_t>(encoder_msg[0]) << 24) +
              (static_cast<uint32_t>(encoder_msg[1]) << 16) +
              (static_cast<uint32_t>(encoder_msg[2]) <<  8) +
              (static_cast<uint32_t>(encoder_msg[3]) <<  0));

  encoder_2 = static_cast<int32_t>(
              (static_cast<uint32_t>(encoder_msg[4]) << 24) +
              (static_cast<uint32_t>(encoder_msg[5]) << 16) +
              (static_cast<uint32_t>(encoder_msg[6]) <<  8) +
              (static_cast<uint32_t>(encoder_msg[7]) <<  0));
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void MD49::enableTimeout()
{
  _serial.write(MD49MessageCreator::createEnableTimeoutMessage());
}

void MD49::disableRegulator()
{
  _serial.write(MD49MessageCreator::createDisableRegulatorMessage());
}

void MD49::setMode(eMD49Mode const mode)
{
  _serial.write(MD49MessageCreator::createSetModeMessage(static_cast<uint8_t>(mode)));
}

void MD49::resetEncoders()
{
  _serial.write(MD49MessageCreator::createResetEncodersMessage());
}
