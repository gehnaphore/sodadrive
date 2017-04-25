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
#include <boost/thread.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "MD49Message.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class pl2303_reactive_descriptor_service
{
public:
  // The native type of a descriptor.
  typedef int native_handle_type;

  // The implementation type of the descriptor.
  class implementation_type
    : private boost::asio::detail::noncopyable
  {
  public:
    // Default constructor.
    implementation_type()
      : descriptor_(-1),
        state_(0)
    {
    }

  private:
    // Only this service will have access to the internal values.
    friend class pl2303_reactive_descriptor_service;
    friend class pl2303_reactive_serial_port_service;

    // The native descriptor representation.
    int descriptor_;

    // The current state of the descriptor.
    boost::asio::detail::descriptor_ops::state_type state_;

    // Per-descriptor data used by the reactor.
    boost::asio::detail::reactor::per_descriptor_data reactor_data_;
  };

  // Constructor.
  BOOST_ASIO_DECL pl2303_reactive_descriptor_service(
      boost::asio::io_service& io_service);

  // Destroy all user-defined handler objects owned by the service.
  BOOST_ASIO_DECL void shutdown_service();

  // Construct a new descriptor implementation.
  BOOST_ASIO_DECL void construct(implementation_type& impl);

  // Move-construct a new descriptor implementation.
  BOOST_ASIO_DECL void move_construct(implementation_type& impl,
      implementation_type& other_impl);

  // Move-assign from another descriptor implementation.
  BOOST_ASIO_DECL void move_assign(implementation_type& impl,
      pl2303_reactive_descriptor_service& other_service,
      implementation_type& other_impl);

  // Destroy a descriptor implementation.
  BOOST_ASIO_DECL void destroy(implementation_type& impl);

  // Assign a native descriptor to a descriptor implementation.
  BOOST_ASIO_DECL boost::system::error_code assign(implementation_type& impl,
      const native_handle_type& native_descriptor,
      boost::system::error_code& ec);

  // Determine whether the descriptor is open.
  bool is_open(const implementation_type& impl) const
  {
    return impl.descriptor_ != -1;
  }

  // Destroy a descriptor implementation.
  BOOST_ASIO_DECL boost::system::error_code close(implementation_type& impl,
      boost::system::error_code& ec);

  // Get the native descriptor representation.
  native_handle_type native_handle(const implementation_type& impl) const
  {
    return impl.descriptor_;
  }

  // Release ownership of the native descriptor representation.
  BOOST_ASIO_DECL native_handle_type release(implementation_type& impl);

  // Cancel all operations associated with the descriptor.
  BOOST_ASIO_DECL boost::system::error_code cancel(implementation_type& impl,
      boost::system::error_code& ec);

  // Perform an IO control command on the descriptor.
  template <typename IO_Control_Command>
  boost::system::error_code io_control(implementation_type& impl,
      IO_Control_Command& command, boost::system::error_code& ec)
  {
    boost::asio::detail::descriptor_ops::ioctl(impl.descriptor_, impl.state_,
        command.name(), static_cast<boost::asio::detail::ioctl_arg_type*>(command.data()), ec);
    return ec;
  }

  // Gets the non-blocking mode of the descriptor.
  bool non_blocking(const implementation_type& impl) const
  {
    return (impl.state_ & boost::asio::detail::descriptor_ops::user_set_non_blocking) != 0;
  }

  // Sets the non-blocking mode of the descriptor.
  boost::system::error_code non_blocking(implementation_type& impl,
      bool mode, boost::system::error_code& ec)
  {
    boost::asio::detail::descriptor_ops::set_user_non_blocking(
        impl.descriptor_, impl.state_, mode, ec);
    return ec;
  }

  // Gets the non-blocking mode of the native descriptor implementation.
  bool native_non_blocking(const implementation_type& impl) const
  {
    return (impl.state_ & boost::asio::detail::descriptor_ops::internal_non_blocking) != 0;
  }

  // Sets the non-blocking mode of the native descriptor implementation.
  boost::system::error_code native_non_blocking(implementation_type& impl,
      bool mode, boost::system::error_code& ec)
  {
    boost::asio::detail::descriptor_ops::set_internal_non_blocking(
        impl.descriptor_, impl.state_, mode, ec);
    return ec;
  }

  // Write some data to the descriptor.
  template <typename ConstBufferSequence>
  size_t write_some(implementation_type& impl,
      const ConstBufferSequence& buffers, boost::system::error_code& ec)
  {
    boost::asio::detail::buffer_sequence_adapter<boost::asio::const_buffer,
        ConstBufferSequence> bufs(buffers);

    return boost::asio::detail::descriptor_ops::sync_write(impl.descriptor_, impl.state_,
        bufs.buffers(), bufs.count(), bufs.all_empty(), ec);
  }

  // Wait until data can be written without blocking.
  size_t write_some(implementation_type& impl,
      const boost::asio::null_buffers&, boost::system::error_code& ec)
  {
    // Wait for descriptor to become ready.
    boost::asio::detail::descriptor_ops::poll_write(impl.descriptor_, impl.state_, ec);

    return 0;
  }

  // Start an asynchronous write. The data being sent must be valid for the
  // lifetime of the asynchronous operation.
  template <typename ConstBufferSequence, typename Handler>
  void async_write_some(implementation_type& impl,
      const ConstBufferSequence& buffers, Handler& handler)
  {
    bool is_continuation =
      boost_asio_handler_cont_helpers::is_continuation(handler);

    // Allocate and construct an operation to wrap the handler.
    typedef boost::asio::detail::descriptor_write_op<ConstBufferSequence, Handler> op;
    typename op::ptr p = { boost::asio::detail::addressof(handler),
      boost_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(impl.descriptor_, buffers, handler);

    BOOST_ASIO_HANDLER_CREATION((p.p, "descriptor", &impl, "async_write_some"));

    start_op(impl, boost::asio::detail::reactor::write_op, p.p, is_continuation, true,
        boost::asio::detail::buffer_sequence_adapter<boost::asio::const_buffer,
          ConstBufferSequence>::all_empty(buffers));
    p.v = p.p = 0;
  }

  // Start an asynchronous wait until data can be written without blocking.
  template <typename Handler>
  void async_write_some(implementation_type& impl,
      const boost::asio::null_buffers&, Handler& handler)
  {
    bool is_continuation =
      boost_asio_handler_cont_helpers::is_continuation(handler);

    // Allocate and construct an operation to wrap the handler.
    typedef boost::asio::detail::reactive_null_buffers_op<Handler> op;
    typename op::ptr p = { boost::asio::detail::addressof(handler),
      boost_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(handler);

    BOOST_ASIO_HANDLER_CREATION((p.p, "descriptor",
          &impl, "async_write_some(null_buffers)"));

    start_op(impl, boost::asio::detail::reactor::write_op, p.p, is_continuation, false, false);
    p.v = p.p = 0;
  }

  // Read some data from the stream. Returns the number of bytes read.
  template <typename MutableBufferSequence>
  size_t read_some(implementation_type& impl,
      const MutableBufferSequence& buffers, boost::system::error_code& ec)
  {
    boost::asio::detail::buffer_sequence_adapter<boost::asio::mutable_buffer,
        MutableBufferSequence> bufs(buffers);

    return boost::asio::detail::descriptor_ops::sync_read(impl.descriptor_, impl.state_,
        bufs.buffers(), bufs.count(), bufs.all_empty(), ec);
  }

  // Wait until data can be read without blocking.
  size_t read_some(implementation_type& impl,
      const boost::asio::null_buffers&, boost::system::error_code& ec)
  {
    // Wait for descriptor to become ready.
    boost::asio::detail::descriptor_ops::poll_read(impl.descriptor_, impl.state_, ec);

    return 0;
  }

  // Start an asynchronous read. The buffer for the data being read must be
  // valid for the lifetime of the asynchronous operation.
  template <typename MutableBufferSequence, typename Handler>
  void async_read_some(implementation_type& impl,
      const MutableBufferSequence& buffers, Handler& handler)
  {
    bool is_continuation =
      boost_asio_handler_cont_helpers::is_continuation(handler);

    // Allocate and construct an operation to wrap the handler.
    typedef boost::asio::detail::descriptor_read_op<MutableBufferSequence, Handler> op;
    typename op::ptr p = { boost::asio::detail::addressof(handler),
      boost_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(impl.descriptor_, buffers, handler);

    BOOST_ASIO_HANDLER_CREATION((p.p, "descriptor", &impl, "async_read_some"));

    start_op(impl, boost::asio::detail::reactor::read_op, p.p, is_continuation, true,
        boost::asio::detail::buffer_sequence_adapter<boost::asio::mutable_buffer,
          MutableBufferSequence>::all_empty(buffers));
    p.v = p.p = 0;
  }

  // Wait until data can be read without blocking.
  template <typename Handler>
  void async_read_some(implementation_type& impl,
      const boost::asio::null_buffers&, Handler& handler)
  {
    bool is_continuation =
      boost_asio_handler_cont_helpers::is_continuation(handler);

    // Allocate and construct an operation to wrap the handler.
    typedef boost::asio::detail::reactive_null_buffers_op<Handler> op;
    typename op::ptr p = { boost::asio::detail::addressof(handler),
      boost_asio_handler_alloc_helpers::allocate(
        sizeof(op), handler), 0 };
    p.p = new (p.v) op(handler);

    BOOST_ASIO_HANDLER_CREATION((p.p, "descriptor",
          &impl, "async_read_some(null_buffers)"));

    start_op(impl, boost::asio::detail::reactor::read_op, p.p, is_continuation, false, false);
    p.v = p.p = 0;
  }

private:
  // Start the asynchronous operation.
  BOOST_ASIO_DECL void start_op(implementation_type& impl, int op_type,
      boost::asio::detail::reactor_op* op, bool is_continuation, bool is_non_blocking, bool noop);

  // The selector that performs event demultiplexing for the service.
  boost::asio::detail::reactor& reactor_;
};

class pl2303_reactive_serial_port_service
{
public:
  // The native type of a serial port.
  typedef pl2303_reactive_descriptor_service::native_handle_type native_handle_type;

  // The implementation type of the serial port.
  typedef pl2303_reactive_descriptor_service::implementation_type implementation_type;

  // Function pointer type for storing a serial port option.
  typedef boost::system::error_code (*store_function_type)(
      const void*, termios&, boost::system::error_code&);

  // Function pointer type for loading a serial port option.
  typedef boost::system::error_code (*load_function_type)(
      void*, const termios&, boost::system::error_code&);


  pl2303_reactive_serial_port_service(
      boost::asio::io_service& io_service)
    : descriptor_service_(io_service)
  {
  }

  void shutdown_service()
  {
    descriptor_service_.shutdown_service();
  }

  boost::system::error_code open(
      pl2303_reactive_serial_port_service::implementation_type& impl,
      const std::string& device, boost::system::error_code& ec)
  {
    if (is_open(impl))
    {
      ec = boost::asio::error::already_open;
      return ec;
    }

    boost::asio::detail::descriptor_ops::state_type state = 0;
    int fd = boost::asio::detail::descriptor_ops::open(device.c_str(),
        O_RDWR | O_NONBLOCK | O_NOCTTY, ec);
    if (fd < 0)
      return ec;

    int s = boost::asio::detail::descriptor_ops::fcntl(fd, F_GETFL, ec);
    if (s >= 0)
      s = boost::asio::detail::descriptor_ops::fcntl(fd, F_SETFL, s | O_NONBLOCK, ec);
    if (s < 0)
    {
      boost::system::error_code ignored_ec;
      boost::asio::detail::descriptor_ops::close(fd, state, ignored_ec);
      return ec;
    }

    // Set up default serial port options.
    termios ios;
    errno = 0;
    s = boost::asio::detail::descriptor_ops::error_wrapper(::tcgetattr(fd, &ios), ec);
    if (s >= 0)
    {
  #if defined(_BSD_SOURCE)
      ::cfmakeraw(&ios);
  #else
      ios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK
          | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
      ios.c_oflag &= ~OPOST;
      ios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      ios.c_cflag &= ~(CSIZE | PARENB);
      ios.c_cflag |= CS8;
  #endif
      ios.c_iflag |= IGNPAR;
      ios.c_cflag |= CREAD | CLOCAL;
      errno = 0;
      s = boost::asio::detail::descriptor_ops::error_wrapper(::tcsetattr(fd, TCSANOW, &ios), ec);
    }
    if (s < 0)
    {
      boost::system::error_code ignored_ec;
      boost::asio::detail::descriptor_ops::close(fd, state, ignored_ec);
      return ec;
    }

    // We're done. Take ownership of the serial port descriptor.
    if (descriptor_service_.assign(impl, fd, ec))
    {
      boost::system::error_code ignored_ec;
      boost::asio::detail::descriptor_ops::close(fd, state, ignored_ec);
    }

    return ec;
  }

  boost::system::error_code do_set_option(
      implementation_type& impl,
      store_function_type store,
      const void* option, boost::system::error_code& ec)
  {
    termios ios;
    errno = 0;
    boost::asio::detail::descriptor_ops::error_wrapper(::tcgetattr(
          descriptor_service_.native_handle(impl), &ios), ec);
    if (ec)
      return ec;

    if (store(option, ios, ec))
      return ec;

    errno = 0;
    boost::asio::detail::descriptor_ops::error_wrapper(::tcsetattr(
          descriptor_service_.native_handle(impl), TCSANOW, &ios), ec);
    return ec;
  }

  boost::system::error_code do_get_option(
      const implementation_type& impl,
      load_function_type load,
      void* option, boost::system::error_code& ec) const
  {
    termios ios;
    errno = 0;
    boost::asio::detail::descriptor_ops::error_wrapper(::tcgetattr(
          descriptor_service_.native_handle(impl), &ios), ec);
    if (ec)
      return ec;

    return load(option, ios, ec);
  }

  // Construct a new serial port implementation.
  void construct(implementation_type& impl)
  {
    descriptor_service_.construct(impl);
  }

  // Move-construct a new serial port implementation.
  void move_construct(implementation_type& impl,
      implementation_type& other_impl)
  {
    descriptor_service_.move_construct(impl, other_impl);
  }

  // Move-assign from another serial port implementation.
  void move_assign(implementation_type& impl,
      pl2303_reactive_serial_port_service& other_service,
      implementation_type& other_impl)
  {
    descriptor_service_.move_assign(impl,
        other_service.descriptor_service_, other_impl);
  }

  // Destroy a serial port implementation.
  void destroy(implementation_type& impl)
  {
    descriptor_service_.destroy(impl);
  }

  // Assign a native descriptor to a serial port implementation.
  boost::system::error_code assign(implementation_type& impl,
      const native_handle_type& native_descriptor,
      boost::system::error_code& ec)
  {
    return descriptor_service_.assign(impl, native_descriptor, ec);
  }

  // Determine whether the serial port is open.
  bool is_open(const implementation_type& impl) const
  {
    return descriptor_service_.is_open(impl);
  }

  // Destroy a serial port implementation.
  boost::system::error_code close(implementation_type& impl,
      boost::system::error_code& ec)
  {
    return descriptor_service_.close(impl, ec);
  }

  // Get the native serial port representation.
  native_handle_type native_handle(implementation_type& impl)
  {
    return descriptor_service_.native_handle(impl);
  }

  // Cancel all operations associated with the serial port.
  boost::system::error_code cancel(implementation_type& impl,
      boost::system::error_code& ec)
  {
    return descriptor_service_.cancel(impl, ec);
  }

  // Set an option on the serial port.
  template <typename SettableSerialPortOption>
  boost::system::error_code set_option(implementation_type& impl,
      const SettableSerialPortOption& option, boost::system::error_code& ec)
  {
    return do_set_option(impl,
        &pl2303_reactive_serial_port_service::store_option<SettableSerialPortOption>,
        &option, ec);
  }

  // Get an option from the serial port.
  template <typename GettableSerialPortOption>
  boost::system::error_code get_option(const implementation_type& impl,
      GettableSerialPortOption& option, boost::system::error_code& ec) const
  {
    return do_get_option(impl,
        &pl2303_reactive_serial_port_service::load_option<GettableSerialPortOption>,
        &option, ec);
  }

  // Send a break sequence to the serial port.
  boost::system::error_code send_break(implementation_type& impl,
      boost::system::error_code& ec)
  {
    errno = 0;
    boost::asio::detail::descriptor_ops::error_wrapper(::tcsendbreak(
          descriptor_service_.native_handle(impl), 0), ec);
    return ec;
  }

  // Write the given data. Returns the number of bytes sent.
  template <typename ConstBufferSequence>
  size_t write_some(implementation_type& impl,
      const ConstBufferSequence& buffers, boost::system::error_code& ec)
  {
    return descriptor_service_.write_some(impl, buffers, ec);
  }

  // Start an asynchronous write. The data being written must be valid for the
  // lifetime of the asynchronous operation.
  template <typename ConstBufferSequence, typename Handler>
  void async_write_some(implementation_type& impl,
      const ConstBufferSequence& buffers, Handler& handler)
  {
    descriptor_service_.async_write_some(impl, buffers, handler);
  }

  // Read some data. Returns the number of bytes received.
  template <typename MutableBufferSequence>
  size_t read_some(implementation_type& impl,
      const MutableBufferSequence& buffers, boost::system::error_code& ec)
  {
    if (boost::asio::detail::descriptor_ops::poll_read(impl.descriptor_, 0, ec) < 0) return 0;
    return descriptor_service_.read_some(impl, buffers, ec);
  }

  // Start an asynchronous read. The buffer for the data being received must be
  // valid for the lifetime of the asynchronous operation.
  template <typename MutableBufferSequence, typename Handler>
  void async_read_some(implementation_type& impl,
      const MutableBufferSequence& buffers, Handler& handler)
  {
    descriptor_service_.async_read_some(impl, buffers, handler);
  }

private:

  // Helper function template to store a serial port option.
  template <typename SettableSerialPortOption>
  static boost::system::error_code store_option(const void* option,
      termios& storage, boost::system::error_code& ec)
  {
    return static_cast<const SettableSerialPortOption*>(option)->store(
        storage, ec);
  }

  // Helper function template to load a serial port option.
  template <typename GettableSerialPortOption>
  static boost::system::error_code load_option(void* option,
      const termios& storage, boost::system::error_code& ec)
  {
    return static_cast<GettableSerialPortOption*>(option)->load(storage, ec);
  }

  // The implementation used for initiating asynchronous operations.
  pl2303_reactive_descriptor_service descriptor_service_;
};

class pl2303_serial_port_service
#if defined(GENERATING_DOCUMENTATION)
  : public boost::asio::io_service::service
#else
  : public boost::asio::detail::service_base<pl2303_serial_port_service>
#endif
{
public:
#if defined(GENERATING_DOCUMENTATION)
  /// The unique service identifier.
  static boost::asio::io_service::id id;
#endif

private:
  // The type of the platform-specific implementation.
#if defined(BOOST_ASIO_HAS_IOCP)
  typedef detail::win_iocp_serial_port_service service_impl_type;
#else
  typedef pl2303_reactive_serial_port_service service_impl_type;
#endif

public:
  /// The type of a serial port implementation.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined implementation_type;
#else
  typedef service_impl_type::implementation_type implementation_type;
#endif

  /// (Deprecated: Use native_handle_type.) The native handle type.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined native_type;
#else
  typedef service_impl_type::native_handle_type native_type;
#endif

  /// The native handle type.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined native_handle_type;
#else
  typedef service_impl_type::native_handle_type native_handle_type;
#endif

  /// Construct a new serial port service for the specified io_service.
  explicit pl2303_serial_port_service(boost::asio::io_service& io_service)
    : boost::asio::detail::service_base<pl2303_serial_port_service>(io_service),
      service_impl_(io_service)
  {
  }

  /// Construct a new serial port implementation.
  void construct(implementation_type& impl)
  {
    service_impl_.construct(impl);
  }

#if defined(BOOST_ASIO_HAS_MOVE) || defined(GENERATING_DOCUMENTATION)
  /// Move-construct a new serial port implementation.
  void move_construct(implementation_type& impl,
      implementation_type& other_impl)
  {
    service_impl_.move_construct(impl, other_impl);
  }

  /// Move-assign from another serial port implementation.
  void move_assign(implementation_type& impl,
      serial_port_service& other_service,
      implementation_type& other_impl)
  {
    service_impl_.move_assign(impl, other_service.service_impl_, other_impl);
  }
#endif // defined(BOOST_ASIO_HAS_MOVE) || defined(GENERATING_DOCUMENTATION)

  /// Destroy a serial port implementation.
  void destroy(implementation_type& impl)
  {
    service_impl_.destroy(impl);
  }

  /// Open a serial port.
  boost::system::error_code open(implementation_type& impl,
      const std::string& device, boost::system::error_code& ec)
  {
    return service_impl_.open(impl, device, ec);
  }

  /// Assign an existing native handle to a serial port.
  boost::system::error_code assign(implementation_type& impl,
      const native_handle_type& handle, boost::system::error_code& ec)
  {
    return service_impl_.assign(impl, handle, ec);
  }

  /// Determine whether the handle is open.
  bool is_open(const implementation_type& impl) const
  {
    return service_impl_.is_open(impl);
  }

  /// Close a serial port implementation.
  boost::system::error_code close(implementation_type& impl,
      boost::system::error_code& ec)
  {
    return service_impl_.close(impl, ec);
  }

  /// (Deprecated: Use native_handle().) Get the native handle implementation.
  native_type native(implementation_type& impl)
  {
    return service_impl_.native_handle(impl);
  }

  /// Get the native handle implementation.
  native_handle_type native_handle(implementation_type& impl)
  {
    return service_impl_.native_handle(impl);
  }

  /// Cancel all asynchronous operations associated with the handle.
  boost::system::error_code cancel(implementation_type& impl,
      boost::system::error_code& ec)
  {
    return service_impl_.cancel(impl, ec);
  }

  /// Set a serial port option.
  template <typename SettableSerialPortOption>
  boost::system::error_code set_option(implementation_type& impl,
      const SettableSerialPortOption& option, boost::system::error_code& ec)
  {
    return service_impl_.set_option(impl, option, ec);
  }

  /// Get a serial port option.
  template <typename GettableSerialPortOption>
  boost::system::error_code get_option(const implementation_type& impl,
      GettableSerialPortOption& option, boost::system::error_code& ec) const
  {
    return service_impl_.get_option(impl, option, ec);
  }

  /// Send a break sequence to the serial port.
  boost::system::error_code send_break(implementation_type& impl,
      boost::system::error_code& ec)
  {
    return service_impl_.send_break(impl, ec);
  }

  /// Write the given data to the stream.
  template <typename ConstBufferSequence>
  std::size_t write_some(implementation_type& impl,
      const ConstBufferSequence& buffers, boost::system::error_code& ec)
  {
    return service_impl_.write_some(impl, buffers, ec);
  }

  /// Start an asynchronous write.
  template <typename ConstBufferSequence, typename WriteHandler>
  BOOST_ASIO_INITFN_RESULT_TYPE(WriteHandler,
      void (boost::system::error_code, std::size_t))
  async_write_some(implementation_type& impl,
      const ConstBufferSequence& buffers,
      BOOST_ASIO_MOVE_ARG(WriteHandler) handler)
  {
    boost::asio::detail::async_result_init<
      WriteHandler, void (boost::system::error_code, std::size_t)> init(
        BOOST_ASIO_MOVE_CAST(WriteHandler)(handler));

    service_impl_.async_write_some(impl, buffers, init.handler);

    return init.result.get();
  }

  /// Read some data from the stream.
  template <typename MutableBufferSequence>
  std::size_t read_some(implementation_type& impl,
      const MutableBufferSequence& buffers, boost::system::error_code& ec)
  {
    return service_impl_.read_some(impl, buffers, ec);
  }

  /// Start an asynchronous read.
  template <typename MutableBufferSequence, typename ReadHandler>
  BOOST_ASIO_INITFN_RESULT_TYPE(ReadHandler,
      void (boost::system::error_code, std::size_t))
  async_read_some(implementation_type& impl,
      const MutableBufferSequence& buffers,
      BOOST_ASIO_MOVE_ARG(ReadHandler) handler)
  {
    boost::asio::detail::async_result_init<
      ReadHandler, void (boost::system::error_code, std::size_t)> init(
        BOOST_ASIO_MOVE_CAST(ReadHandler)(handler));

    service_impl_.async_read_some(impl, buffers, init.handler);

    return init.result.get();
  }

private:
  // Destroy all user-defined handler objects owned by the service.
  void shutdown_service()
  {
    service_impl_.shutdown_service();
  }

  // The platform-specific implementation.
  service_impl_type service_impl_;
};

class pl2303_serial_port : public boost::asio::basic_serial_port<pl2303_serial_port_service> {
public:
  explicit pl2303_serial_port(boost::asio::io_service& io_service)
    : basic_serial_port<pl2303_serial_port_service>(io_service)
  {
  }

  explicit pl2303_serial_port(boost::asio::io_service& io_service,
      const char* device)
    : basic_serial_port<pl2303_serial_port_service>(io_service,device)
  {
  }

  explicit pl2303_serial_port(boost::asio::io_service& io_service,
      const std::string& device)
    : basic_serial_port<pl2303_serial_port_service>(io_service,device)
  {
  }
};

class Serial
{
public:

  Serial  (std::string const &dev_node, size_t const baud_rate, bool const show_debug_out);
  ~Serial ();

  void        write (MD49Message  const &msg      );
  MD49Message read  (size_t       const num_bytes );

private:

  std::string               _dev;
  boost::asio::io_service   _io_service;
  boost::asio::io_service::work _work;
	boost::thread             _io_thread;
  pl2303_serial_port        _serial_port;
  boost::asio::deadline_timer deadline;
  size_t                    _baudRate;

  bool                      _show_debug_out;
  bool                      _first_read_done;

  size_t                    _tx_msg_cnt,
                            _rx_msg_cnt;

  void checkDeadline();
  void openPort();
};

#endif /* RPI_SRC_WHEEL_CONTROLLER_SERIAL_H_ */
