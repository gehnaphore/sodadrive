/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

#ifndef RPI_SRC_WHEEL_CONTROLLER_SERIAL_H_
#define RPI_SRC_WHEEL_CONTROLLER_SERIAL_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>

#include <boost/asio.hpp>

#include "MD49Message.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Serial
{
public:

  Serial  (std::string const &dev_node, size_t const baud_rate, bool const show_debug_out);
  ~Serial ();

  void        write (MD49Message  const &msg      );
  MD49Message read  (size_t       const num_bytes );

private:

  boost::asio::io_service   _io_service;
  boost::asio::serial_port  _serial_port;

  bool                      _show_debug_out;

  size_t                    _tx_msg_cnt,
                            _rx_msg_cnt;
};

#endif /* RPI_SRC_WHEEL_CONTROLLER_SERIAL_H_ */
