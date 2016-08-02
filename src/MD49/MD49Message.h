/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_MD49_MD49MESSAGE_H_
#define RPI_SRC_MD49_MD49MESSAGE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdint.h>

#include <vector>
#include <iostream>

/**************************************************************************************
 * TYPEDEFS
 **************************************************************************************/

typedef std::vector<uint8_t> MD49Message;

/**************************************************************************************
 * PUBLIC PROTOTYPES
 **************************************************************************************/

std::ostream & operator << (std::ostream &os, MD49Message const &msg);

#endif /* RPI_SRC_MD49_MD49MESSAGE_H_ */
