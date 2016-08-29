/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_WHEEL_CONTROLLER_MD49_H_
#define RPI_SRC_WHEEL_CONTROLLER_MD49_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Serial.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MD49
{

public:

  MD49(Serial &serial);

  void setSpeed1  (int8_t const speed_1);
  void setSpeed2  (int8_t const speed_2);
  void getEncoders(int32_t &encoder_1, int32_t &encoder_2);

private:

  Serial &_serial;

  typedef enum
  {
    MODE_1 = 0,
    MODE_2 = 1,
    MODE_3 = 2,
    MODE_4 = 3
  } eMD49Mode;

  void enableTimeout    (                     );
  void disableRegulator (                     );
  void setMode          (eMD49Mode const mode );
  void resetEncoders    (                     );

};


#endif /* RPI_SRC_WHEEL_CONTROLLER_MD49_H_ */
