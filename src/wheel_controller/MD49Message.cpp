/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "MD49Message.h"

#include <iomanip>

/**************************************************************************************
 * PUBLIC FUNCTIONS
 **************************************************************************************/

std::ostream & operator << (std::ostream &os, MD49Message const &msg)
{
  for(size_t i = 0; i < msg.size(); i++)
  {
    os << std::hex << std::setw(2) << std::setfill('0') << static_cast<size_t>(msg[i]) << " " << std::dec;
  }
  return os;
}
