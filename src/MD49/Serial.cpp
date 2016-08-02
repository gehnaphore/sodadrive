/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Serial.h"

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

Serial::Serial(std::string const &dev_node, size_t const baud_rate)
: _serial_port(_io_service, dev_node)
{
  _serial_port.set_option(boost::asio::serial_port_base::baud_rate      (baud_rate                                        ));
  _serial_port.set_option(boost::asio::serial_port_base::character_size (8                                                ));
  _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none));
  _serial_port.set_option(boost::asio::serial_port_base::parity         (boost::asio::serial_port_base::parity::none      ));
  _serial_port.set_option(boost::asio::serial_port_base::stop_bits      (boost::asio::serial_port_base::stop_bits::one    ));
}

Serial::~Serial()
{

}

void Serial::write(MD49Message const &msg)
{
  boost::asio::write(_serial_port, boost::asio::buffer(msg));
}

MD49Message Serial::read(size_t const num_bytes)
{
  MD49Message msg(num_bytes);

  boost::asio::read(_serial_port, boost::asio::buffer(msg, num_bytes));

  return msg;
}
