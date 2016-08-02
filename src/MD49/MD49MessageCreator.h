/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_MD49MESSAGECREATOR_H_
#define RPI_SRC_MD49_MD49MESSAGECREATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <vector>

#include "../MD49/MD49CommandId.h"

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef std::vector<uint8_t> MD49Message;

/**************************************************************************************
 * PUBLIC PROTOTYPES
 **************************************************************************************/

class MD49MessageCreator
{

public:

  static MD49Message createGetSpeed1Message       (                           );
  static MD49Message createGetSpeed2Message       (                           );
  static MD49Message createGetEncoder1Message     (                           );
  static MD49Message createGetEncoder2Message     (                           );
  static MD49Message createGetEncodersMessage     (                           );
  static MD49Message createGetVoltsMessage        (                           );
  static MD49Message createGetCurrent1Message     (                           );
  static MD49Message createGetCurrent2Message     (                           );
  static MD49Message createGetVersionMessage      (                           );
  static MD49Message createGetAccelerationMessage (                           );
  static MD49Message createGetModeMessage         (                           );
  static MD49Message createGetVIMessage           (                           );
  static MD49Message createGetErrorMessage        (                           );
  static MD49Message createSetSpeed1Message       (uint8_t const speed_1      );
  static MD49Message createSetSpeed2Message       (uint8_t const speed_2      );
  static MD49Message createSetAccelerationMessage (uint8_t const acceleration );
  static MD49Message createSetModeMessage         (uint8_t const mode         );
  static MD49Message createResetEncodersMessage   (                           );
  static MD49Message createDisableRegulatorMessage(                           );
  static MD49Message createEnableRegulatorMessage (                           );
  static MD49Message createDisableTimeoutMessage  (                           );
  static MD49Message createEnableTimeoutMessage   (                           );

private:

  MD49MessageCreator() { }
  MD49MessageCreator(MD49MessageCreator const &other) { }

  static MD49Message createBaseMessage(MD49CommandId::eCommandId const md49_command_id, uint8_t const payload_size);

};

#endif /* RPI_SRC_MD49_MD49MESSAGECREATOR_H_ */
