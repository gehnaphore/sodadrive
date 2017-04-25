/**
 * \author Alexander Entinger, MSC / LXRobotics
 * \copyright LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "Serial.h"

#include <iostream>

pl2303_reactive_descriptor_service::pl2303_reactive_descriptor_service(
    boost::asio::io_service& io_service)
  : reactor_(boost::asio::use_service<boost::asio::detail::reactor>(io_service))
{
  reactor_.init_task();
}

void pl2303_reactive_descriptor_service::shutdown_service()
{
}

void pl2303_reactive_descriptor_service::construct(
    pl2303_reactive_descriptor_service::implementation_type& impl)
{
  impl.descriptor_ = -1;
  impl.state_ = 0;
}

void pl2303_reactive_descriptor_service::move_construct(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    pl2303_reactive_descriptor_service::implementation_type& other_impl)
{
  impl.descriptor_ = other_impl.descriptor_;
  other_impl.descriptor_ = -1;

  impl.state_ = other_impl.state_;
  other_impl.state_ = 0;

  reactor_.move_descriptor(impl.descriptor_,
      impl.reactor_data_, other_impl.reactor_data_);
}

void pl2303_reactive_descriptor_service::move_assign(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    pl2303_reactive_descriptor_service& other_service,
    pl2303_reactive_descriptor_service::implementation_type& other_impl)
{
  destroy(impl);

  impl.descriptor_ = other_impl.descriptor_;
  other_impl.descriptor_ = -1;

  impl.state_ = other_impl.state_;
  other_impl.state_ = 0;

  other_service.reactor_.move_descriptor(impl.descriptor_,
      impl.reactor_data_, other_impl.reactor_data_);
}

void pl2303_reactive_descriptor_service::destroy(
    pl2303_reactive_descriptor_service::implementation_type& impl)
{
  if (is_open(impl))
  {
    BOOST_ASIO_HANDLER_OPERATION(("descriptor", &impl, "close"));

    reactor_.deregister_descriptor(impl.descriptor_, impl.reactor_data_,
        (impl.state_ & boost::asio::detail::descriptor_ops::possible_dup) == 0);
  }

  boost::system::error_code ignored_ec;
  boost::asio::detail::descriptor_ops::close(impl.descriptor_, impl.state_, ignored_ec);
}

boost::system::error_code pl2303_reactive_descriptor_service::assign(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    const native_handle_type& native_descriptor, boost::system::error_code& ec)
{
  if (is_open(impl))
  {
    ec = boost::asio::error::already_open;
    return ec;
  }

  if (int err = reactor_.register_descriptor(
        native_descriptor, impl.reactor_data_))
  {
    ec = boost::system::error_code(err,
        boost::asio::error::get_system_category());
    return ec;
  }

  impl.descriptor_ = native_descriptor;
  impl.state_ = boost::asio::detail::descriptor_ops::possible_dup;
  ec = boost::system::error_code();
  return ec;
}

boost::system::error_code pl2303_reactive_descriptor_service::close(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    boost::system::error_code& ec)
{
  if (is_open(impl))
  {
    BOOST_ASIO_HANDLER_OPERATION(("descriptor", &impl, "close"));

    reactor_.deregister_descriptor(impl.descriptor_, impl.reactor_data_,
        (impl.state_ & boost::asio::detail::descriptor_ops::possible_dup) == 0);
  }

  boost::asio::detail::descriptor_ops::close(impl.descriptor_, impl.state_, ec);

  // The descriptor is closed by the OS even if close() returns an error.
  //
  // (Actually, POSIX says the state of the descriptor is unspecified. On
  // Linux the descriptor is apparently closed anyway; e.g. see
  //   http://lkml.org/lkml/2005/9/10/129
  // We'll just have to assume that other OSes follow the same behaviour.)
  construct(impl);

  return ec;
}

pl2303_reactive_descriptor_service::native_handle_type
pl2303_reactive_descriptor_service::release(
    pl2303_reactive_descriptor_service::implementation_type& impl)
{
  native_handle_type descriptor = impl.descriptor_;

  if (is_open(impl))
  {
    BOOST_ASIO_HANDLER_OPERATION(("descriptor", &impl, "release"));

    reactor_.deregister_descriptor(impl.descriptor_, impl.reactor_data_, false);
    construct(impl);
  }

  return descriptor;
}

boost::system::error_code pl2303_reactive_descriptor_service::cancel(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    boost::system::error_code& ec)
{
  if (!is_open(impl))
  {
    ec = boost::asio::error::bad_descriptor;
    return ec;
  }

  BOOST_ASIO_HANDLER_OPERATION(("descriptor", &impl, "cancel"));

  reactor_.cancel_ops(impl.descriptor_, impl.reactor_data_);
  ec = boost::system::error_code();
  return ec;
}

void pl2303_reactive_descriptor_service::start_op(
    pl2303_reactive_descriptor_service::implementation_type& impl,
    int op_type, boost::asio::detail::reactor_op* op, bool is_continuation,
    bool is_non_blocking, bool noop)
{
  if (!noop)
  {
    if ((impl.state_ & boost::asio::detail::descriptor_ops::non_blocking) ||
        boost::asio::detail::descriptor_ops::set_internal_non_blocking(
          impl.descriptor_, impl.state_, true, op->ec_))
    {
      reactor_.start_op(op_type, impl.descriptor_,
          impl.reactor_data_, op, is_continuation, is_non_blocking);
      return;
    }
  }

  reactor_.post_immediate_completion(op, is_continuation);
}


/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

Serial::Serial(std::string const &dev_node, size_t const baud_rate, bool const show_debug_out)
: _dev            (dev_node),
  _work           (_io_service),
  _io_thread      (boost::bind(&boost::asio::io_service::run, &_io_service)),
  _serial_port    (_io_service),
  deadline        (_io_service),
  _baudRate       (baud_rate),
  _show_debug_out (show_debug_out),
  _first_read_done(false),
  _tx_msg_cnt     (0),
  _rx_msg_cnt     (0)
{
  openPort();
  deadline.expires_at(boost::posix_time::pos_infin);
  checkDeadline();
}

Serial::~Serial()
{
}

void Serial::openPort() {
  boost::system::error_code ec;
  _serial_port.open(_dev,ec);
  _serial_port.set_option(boost::asio::serial_port_base::baud_rate      (_baudRate                                        ));
  _serial_port.set_option(boost::asio::serial_port_base::character_size (8                                                ));
  _serial_port.set_option(boost::asio::serial_port_base::flow_control   (boost::asio::serial_port_base::flow_control::none));
  _serial_port.set_option(boost::asio::serial_port_base::parity         (boost::asio::serial_port_base::parity::none      ));
  _serial_port.set_option(boost::asio::serial_port_base::stop_bits      (boost::asio::serial_port_base::stop_bits::one    ));
}

void Serial::checkDeadline()
{
  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled. This allows the blocked
    // connect(), read_line() or write_line() functions to return.
    boost::system::error_code ignored_ec;
    std::cout << "Timeout, reopening" << std::endl;
    _serial_port.close();
    openPort();


    // There is no longer an active deadline. The expiry is set to positive
    // infinity so that the actor takes no action until a new deadline is set.
    deadline.expires_at(boost::posix_time::pos_infin);
  }

  // Put the actor back to sleep.
  deadline.async_wait(boost::bind(&Serial::checkDeadline, this));
}

void Serial::write(MD49Message const &msg)
{
  boost::asio::write(_serial_port, boost::asio::buffer(msg));

  if(_show_debug_out)
  {
    std::cout << "TX #" << _tx_msg_cnt << " " << msg << std::endl;
    _tx_msg_cnt++;
  }
}

MD49Message Serial::read(size_t const num_bytes)
{
  MD49Message msg(num_bytes);
  boost::posix_time::time_duration timeout = boost::posix_time::milliseconds(500);

  if (!_first_read_done) {
    _first_read_done = true;
  }

  boost::system::error_code ec;
  deadline.expires_from_now(timeout);
  boost::asio::read(_serial_port, boost::asio::buffer(msg, num_bytes), ec);

  if(_show_debug_out)
  {
    std::cout << "RX #" << _rx_msg_cnt << " " << msg << std::endl;
    _rx_msg_cnt++;
  }

  return msg;
}
