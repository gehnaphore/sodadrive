/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_MD49COMMANDID_H_
#define RPI_SRC_MD49_MD49COMMANDID_H_

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MD49CommandId
{

public:

  typedef enum
  {
    GET_SPEED_1       = 0x21,
    GET_SPEED_2       = 0x22,
    GET_ENCODER_1     = 0x23,
    GET_ENCODER_2     = 0x24,
    GET_ENCODERS      = 0x25,
    GET_VOLTS         = 0x26,
    GET_CURRENT_1     = 0x27,
    GET_CURRENT_2     = 0x28,
    GET_VERSION       = 0x29,
    GET_ACCELERATION  = 0x2A,
    GET_MODE          = 0x2B,
    GET_VI            = 0x2C,
    GET_ERROR         = 0x2D,

    SET_SPEED_1       = 0x31,
    SET_SPEED_2       = 0x32,
    SET_ACCELERATION  = 0x33,
    SET_MODE          = 0x34,

    RESET_ENCODERS    = 0x35,
    DISABLE_REGULATOR = 0x36,
    ENABLE_REGULATOR  = 0x37,
    DISABLE_TIMEOUT   = 0x38,
    ENABLE_TIMEOUT    = 0x39
  } eCommandId;

private:

  MD49CommandId() { }
  MD49CommandId(MD49CommandId const &other) { }

};

#endif /* RPI_SRC_MD49_MD49COMMANDID_H_ */
