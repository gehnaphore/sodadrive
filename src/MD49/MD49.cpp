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
  setMode         (MODE_2 );
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

void MD49::getEncoders(uint32_t &encoder_1, uint32_t &encoder_2)
{
  _serial.write(MD49MessageCreator::createGetEncodersMessage());

  MD49Message const encoder_msg = _serial.read(8);

  memcpy(&encoder_1, &encoder_msg[0],                     sizeof(encoder_1));
  memcpy(&encoder_2, &encoder_msg[0 + sizeof(encoder_1)], sizeof(encoder_2));
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
  _serial.write(MD49MessageCreator::createDisableTimeoutMessage());
}

void MD49::setMode(eMD49Mode const mode)
{
  _serial.write(MD49MessageCreator::createSetModeMessage(static_cast<uint8_t>(mode)));
}

void MD49::resetEncoders()
{
  _serial.write(MD49MessageCreator::createResetEncodersMessage());
}
