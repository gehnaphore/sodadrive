/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "../MD49/MD49MessageCreator.h"

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint8_t const SYNC_BYTE      = 0x00;
static uint8_t const PAYLOAD_START  = 2;

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

MD49Message MD49MessageCreator::createGetSpeed1Message()
{
  return createBaseMessage(MD49CommandId::GET_SPEED_1, 0);
}

MD49Message MD49MessageCreator::createGetSpeed2Message()
{
  return createBaseMessage(MD49CommandId::GET_SPEED_2, 0);
}

MD49Message MD49MessageCreator::createGetEncoder1Message()
{
  return createBaseMessage(MD49CommandId::GET_ENCODER_1, 0);
}

MD49Message MD49MessageCreator::createGetEncoder2Message()
{
  return createBaseMessage(MD49CommandId::GET_ENCODER_2, 0);
}

MD49Message MD49MessageCreator::createGetEncodersMessage()
{
  return createBaseMessage(MD49CommandId::GET_ENCODERS, 0);
}

MD49Message MD49MessageCreator::createGetVoltsMessage()
{
  return createBaseMessage(MD49CommandId::GET_VOLTS, 0);
}

MD49Message MD49MessageCreator::createGetCurrent1Message()
{
  return createBaseMessage(MD49CommandId::GET_CURRENT_1, 0);
}

MD49Message MD49MessageCreator::createGetCurrent2Message()
{
  return createBaseMessage(MD49CommandId::GET_CURRENT_2, 0);
}

MD49Message MD49MessageCreator::createGetVersionMessage()
{
  return createBaseMessage(MD49CommandId::GET_VERSION, 0);
}

MD49Message MD49MessageCreator::createGetAccelerationMessage()
{
  return createBaseMessage(MD49CommandId::GET_ACCELERATION, 0);
}

MD49Message MD49MessageCreator::createGetModeMessage()
{
  return createBaseMessage(MD49CommandId::GET_MODE, 0);
}

MD49Message MD49MessageCreator::createGetVIMessage()
{
  return createBaseMessage(MD49CommandId::GET_VI, 0);
}

MD49Message MD49MessageCreator::createGetErrorMessage()
{
  return createBaseMessage(MD49CommandId::GET_ERROR, 0);
}

MD49Message MD49MessageCreator::createSetSpeed1Message(uint8_t const speed_1)
{
  MD49Message msg                 = createBaseMessage(MD49CommandId::SET_SPEED_1, 1);
              msg[PAYLOAD_START]  = speed_1;
  return      msg;
}

MD49Message MD49MessageCreator::createSetSpeed2Message(uint8_t const speed_2)
{
  MD49Message msg                 = createBaseMessage(MD49CommandId::SET_SPEED_2, 1);
              msg[PAYLOAD_START]  = speed_2;
  return      msg;
}

MD49Message MD49MessageCreator::createSetAccelerationMessage(uint8_t const acceleration)
{
  MD49Message msg                 = createBaseMessage(MD49CommandId::SET_ACCELERATION, 1);
              msg[PAYLOAD_START]  = acceleration;
  return      msg;
}

MD49Message MD49MessageCreator::createSetModeMessage(uint8_t const mode)
{
  MD49Message msg                 = createBaseMessage(MD49CommandId::SET_MODE, 1);
              msg[PAYLOAD_START]  = mode;
  return      msg;
}

MD49Message MD49MessageCreator::createResetEncodersMessage()
{
  return createBaseMessage(MD49CommandId::RESET_ENCODERS, 0);
}

MD49Message MD49MessageCreator::createDisableRegulatorMessage()
{
  return createBaseMessage(MD49CommandId::DISABLE_REGULATOR, 0);
}

MD49Message MD49MessageCreator::createEnableRegulatorMessage()
{
  return createBaseMessage(MD49CommandId::ENABLE_REGULATOR, 0);
}

MD49Message MD49MessageCreator::createDisableTimeoutMessage()
{
  return createBaseMessage(MD49CommandId::DISABLE_TIMEOUT, 0);
}

MD49Message MD49MessageCreator::createEnableTimeoutMessage()
{
  return createBaseMessage(MD49CommandId::ENABLE_TIMEOUT, 0);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

MD49Message MD49MessageCreator::createBaseMessage(MD49CommandId::eCommandId const md49_command_id, uint8_t const payload_size)
{
  MD49Message msg(1 + 1 + payload_size); /* 1 SYNC Byte + 1 Command Id + payload_size Payload Bytes */

  msg[0] = SYNC_BYTE;
  msg[1] = static_cast<uint8_t>(md49_command_id);

  return msg;
}
