/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "MD49MessageCreator.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

MD49Message MD49MessageCreator::createGetSpeed1Message       ();
MD49Message MD49MessageCreator::createGetSpeed2Message       ();
MD49Message MD49MessageCreator::createGetEncoder1Message     ();
MD49Message MD49MessageCreator::createGetEncoder2Message     ();
MD49Message MD49MessageCreator::createGetEncodersMessage     ();
MD49Message MD49MessageCreator::createGetVoltsMessage        ();
MD49Message MD49MessageCreator::createGetCurrent1Message     ();
MD49Message MD49MessageCreator::createGetCurrent2Message     ();
MD49Message MD49MessageCreator::createGetVersionMessage      ();
MD49Message MD49MessageCreator::createGetAccelerationMessage ();
MD49Message MD49MessageCreator::createGetModeMessage         ();
MD49Message MD49MessageCreator::createGetVIMessage           ();
MD49Message MD49MessageCreator::createGetErrorMessage        ();
MD49Message MD49MessageCreator::createSetSpeed1Message       (uint8_t const speed_1);
MD49Message MD49MessageCreator::createSetSpeed2Message       (uint8_t const speed_2);
MD49Message MD49MessageCreator::createSetAccelerationMessage (uint8_t const acceleration);
MD49Message MD49MessageCreator::createSetModeMessage         (uint8_t const mode);
MD49Message MD49MessageCreator::createResetEncodersMessage   ();
MD49Message MD49MessageCreator::createDisableRegulatorMessage();
MD49Message MD49MessageCreator::createEnableRegulatorMessage ();
MD49Message MD49MessageCreator::createDisableTimeoutMessage  ();
MD49Message MD49MessageCreator::createEnableTimeoutMessage   ();

MD49Message MD49MessageCreator::createBaseMessage(MD49CommandId::eCommandId const md49_command_id, uint8_t const payload_size)
{
  MD49Message msg(1 + 1 + payload_size); /* 1 SYNC Byte + 1 Command Id + payload_size Payload Bytes */

  msg[0] = 0x00;
  msg[1] = static_cast<uint8_t>(md49_command_id);

  return msg;
}
