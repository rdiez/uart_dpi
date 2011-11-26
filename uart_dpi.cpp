
/* Version 0.80 beta, November 2011.

   See the README file for information about this module.

   During development, use compiler flag -DDEBUG in order to enable assertions.
   
   Copyright (c) 2011 R. Diez
                                                             
   This source file may be used and distributed without        
   restriction provided that this copyright statement is not   
   removed from the file and that any derivative work contains 
   the original copyright notice and the associated disclaimer.
                                                             
   This source file is free software; you can redistribute it  
   and/or modify it under the terms of the GNU Lesser General  
   Public License version 3 as published by the Free Software Foundation.
                                                             
   This source is distributed in the hope that it will be      
   useful, but WITHOUT ANY WARRANTY; without even the implied  
   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR     
   PURPOSE.  See the GNU Lesser General Public License for more
   details.                                                    
                                                             
   You should have received a copy of the GNU Lesser General   
   Public License along with this source; if not, download it  
   from http://www.gnu.org/licenses/
*/

#include "Vminsoc_bench_core.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>

#include <stdexcept>
#include <sstream>


// We may have more error codes in the future, that's why the success value is zero.
// It would be best to return the error message as a string, but Verilog
// does not have good support for variable-length strings.
static const int RET_SUCCESS = 0;
static const int RET_FAILURE = 1;

static const char ERROR_MSG_PREFIX[] = "Error in the UART DPI module: ";


class uart_dpi
{
private:
  uint16_t m_listening_tcp_port;
  int      m_listening_socket;  // -1 means no listening socket.
  bool     m_listen_on_local_addr_only;

  unsigned m_receive_buffer_size;
  unsigned m_transmit_buffer_size;

  uint8_t * m_receive_buffer;
  uint8_t * m_transmit_buffer;

  unsigned m_receive_write_pointer;
  unsigned m_receive_read_pointer;
  unsigned m_transmit_write_pointer;
  unsigned m_transmit_read_pointer;

  bool m_print_informational_messages;
  std::string m_informational_message_prefix;
  bool m_listening_message_already_printed;

  std::string m_welcome_message;
  int m_welcome_message_pos;  // -1 means no message or already sent for this connection.

  int m_connectionSocket;  // -1 means no connection.

  bool is_receive_buffer_empty ( void );
  bool is_receive_buffer_full ( void );
  void enqueue_receive_byte ( uint8_t character );
  uint8_t deque_receive_byte ( void );
  int get_received_byte_count ( void );
  bool is_transmit_buffer_full ( void );
  bool is_transmit_buffer_empty ( void );
  uint8_t deque_transmit_byte ( void );
  
  void close_current_connection ( void );
  void close_listening_socket ( void );
  void create_listening_socket ( void );
  void accept_connection ( void );
  void accept_eventual_incoming_connection ( void );
  void send_byte ( uint8_t data );

  void transmit_data ( void );
  void receive_data ( void );

public:
  uart_dpi ( int tcp_port,
             unsigned char listen_on_local_addr_only,
             int transmit_buffer_size,
             int receive_buffer_size,
             const char * welcome_message,
             unsigned char print_informational_messages,
             const char * informational_message_prefix );
  ~uart_dpi ( void );

  void send_char ( char character );
  char receive ( void );
  void tick ( int * received_byte_count );
};


static std::string get_error_message ( const char * const prefix_msg,
                                       const int errno_val )
{
  std::ostringstream str;
  
  if ( prefix_msg != NULL )
    str << prefix_msg;
  
  str << "Error code " << errno_val << ": ";

  char buffer[ 2048 ];

  #if (_POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600) && ! _GNU_SOURCE
  #error "The call to strerror_r() below will not compile properly. The easiest thing to do is to define _GNU_SOURCE when compiling this module."
  #endif
  
  const char * const err_msg = strerror_r( errno_val, buffer, sizeof(buffer) );
  
  if ( err_msg == NULL )
  {
    str << "<no error message available>";
  }
  else
  {
    // Always terminate the string, just in case. Note that the string
    // may not actually be in the buffer, see the strerror_r() documentation.
    buffer[ sizeof( buffer ) / sizeof( buffer[0] ) - 1 ] = '\0';
    assert( strlen( buffer ) < sizeof( buffer ) );
    
    str << err_msg;
  }
  
  return str.str();
}


void uart_dpi::close_current_connection ( void )
{
  assert( m_connectionSocket != -1 );
  
  if ( -1 == close( m_connectionSocket ) )
    assert( false );
  
  m_connectionSocket = -1;
}


void uart_dpi::send_byte ( const uint8_t data )
{
  if ( -1 == send( m_connectionSocket,
                   &data,
                   sizeof(data),
                   0  // No special flags. MSG_DONTWAIT not needed, for the socket is in SOCK_NONBLOCK mode.
                 ) )
  {
    throw std::runtime_error( get_error_message( "Error sending data: ", errno ) );
  }
}


static std::string ip_address_to_text ( const in_addr * const addr )
{
  char ip_addr_buffer[80];
  
  const char * const str = inet_ntop( AF_INET,
                                      addr,
                                      ip_addr_buffer,
                                      sizeof(ip_addr_buffer) );
  if ( str == NULL )
  {
    throw std::runtime_error( get_error_message( "Error formatting the IP address: ", errno ) );
  }

  assert( strlen(str) <= strlen("123.123.123.123") );
  assert( strlen(str) <= sizeof(ip_addr_buffer) );

  return str;
}


void uart_dpi::close_listening_socket ( void )
{
  assert( m_listening_socket != -1 );

  if ( -1 == close( m_listening_socket ) )
    assert( false );

  m_listening_socket = -1;
}


void uart_dpi::create_listening_socket ( void )
{
  assert( m_listening_socket == -1 );

  m_listening_socket = socket( PF_INET,
                               SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC,
                               0 );

  if ( m_listening_socket == -1 )
  {
    throw std::runtime_error( get_error_message( "Error creating the listening socket: ", errno ) );
  }

  try
  {
    // If this process terminates abruptly, the TCP/IP stack does not release
    // the listening ports immediately, at least under Linux (I've seen comments
    // about this issue under Windows too). Therefore, if you restart the simulation
    // whithin a few seconds, you'll get an annoying "address already in use" error message.
    // The SO_REUSEADDR prevents this from happening.
    const int set_reuse_to_yes = 1;
    if ( setsockopt( m_listening_socket,
                     SOL_SOCKET,
                     SO_REUSEADDR,
                     &set_reuse_to_yes,
                     sizeof(set_reuse_to_yes) ) == -1 )
    {
      throw std::runtime_error( get_error_message( "Error setting the listen socket options: ", errno ) );
    }
    
    sockaddr_in addr;
    memset( &addr, 0, sizeof(addr) );
    addr.sin_family = AF_INET;
    addr.sin_port = htons( m_listening_tcp_port );
    addr.sin_addr.s_addr = ntohl( m_listen_on_local_addr_only ? INADDR_LOOPBACK : INADDR_ANY );

    if ( bind( m_listening_socket,
               (struct sockaddr *)&addr,
               sizeof(addr) ) == -1 )
    {
      throw std::runtime_error( get_error_message( "Error binding the socket: ", errno ) );
    }

    // The listening IP address and listening port do not change, so print this information
    // only once at the beginning. Printing the message again just clutters
    // the screen with unnecessary information.
    if ( !m_listening_message_already_printed )
    {
      m_listening_message_already_printed = true;

      if ( m_print_informational_messages )
      {
        const std::string addr_str = ip_address_to_text( &addr.sin_addr );
        
        printf( "%sListening on IP address %s (%s), TCP port %d.\n",
                m_informational_message_prefix.c_str(),
                addr_str.c_str(),
                m_listen_on_local_addr_only ? "local only" : "all",
                m_listening_tcp_port );
        fflush( stdout );
      }
    }
  
    if ( listen( m_listening_socket, 1 ) == -1 )
    {
      throw std::runtime_error( get_error_message( "Error listening on the socket: ", errno ) );
    }
  }
  catch ( ... )
  {
    close_listening_socket();
    throw;
  }
}


void uart_dpi::accept_connection ( void )
{
  assert( m_listening_socket != -1 );

  pollfd polledFd;

  polledFd.fd      = m_listening_socket;
  polledFd.events  = POLLIN | POLLERR;
  polledFd.revents = 0;

  const int pollRes = poll( &polledFd, 1, 0 );

  if ( pollRes == 0 )
  {
    // No incoming connection is yet there.
    return;
  }
  
  if ( pollRes == -1 )
  {
    throw std::runtime_error( get_error_message( "Error polling the listening socket: ", errno ) );
  }

  assert( pollRes == 1 );

  if ( m_print_informational_messages )
  {
    // printf( "%sPoll result flags: 0x%02X\n", polledFd.revents, m_informational_message_prefix.c_str() );
    // fflush( stdout );
  }
    
  sockaddr_in remoteAddr;
  socklen_t remoteAddrLen = sizeof( remoteAddr );

  const int connectionSocket = accept4( m_listening_socket,
                                        (sockaddr *) &remoteAddr,
                                        &remoteAddrLen,
                                        SOCK_NONBLOCK | SOCK_CLOEXEC );

  // Any errors accepting a connection are considered non-critical and do not normally stop the simulation,
  // as the remote client can try to reconnect at a later point in time.
  try
  {
    if ( connectionSocket == -1 )
    {
      throw std::runtime_error( get_error_message( NULL, errno ) );
    }
    
    if ( remoteAddrLen > sizeof( remoteAddr ) )
    {
      throw std::runtime_error( "The address buffer is too small." );
    }

    if ( m_print_informational_messages )
    {
      const std::string addr_str = ip_address_to_text( &remoteAddr.sin_addr );
      
      printf( "%sAccepted an incoming connection from IP address %s, TCP port %d.\n",
              m_informational_message_prefix.c_str(),
              addr_str.c_str(),
              ntohs( remoteAddr.sin_port ) );
      fflush( stdout );
    }
  }
  catch ( const std::exception & e )
  {
    fprintf( stderr,
             "%sError accepting a connection on the listening socket: %s\n",
             ERROR_MSG_PREFIX,
             e.what() );
    fflush( stderr );
    
    if ( connectionSocket != -1 )
    {
      if ( -1 == close( connectionSocket ) )
        assert( false );
    }

    return;
  }

  m_connectionSocket = connectionSocket;
  m_welcome_message_pos = m_welcome_message.empty() ? -1 : 0;

  // If somebody else attempts to connect, he should get an error straight away.
  // However, if the listening socket is still active, the client will land in the accept queue
  // and he'll hopefully time-out eventually.
  close_listening_socket();
}


void uart_dpi::accept_eventual_incoming_connection ( void )
{
  if ( m_connectionSocket == -1 )
  {
    if ( m_listening_socket == -1 )
    {
      create_listening_socket();
    }

    accept_connection();
  }
}


bool uart_dpi::is_receive_buffer_empty ( void )
{
  return m_receive_read_pointer == m_receive_write_pointer;
}

bool uart_dpi::is_receive_buffer_full ( void )
{
  return ((m_receive_write_pointer + 1) % m_receive_buffer_size) == m_receive_read_pointer;
}

void uart_dpi::enqueue_receive_byte ( const uint8_t character )
{
  assert( !is_receive_buffer_full() );

  m_receive_buffer[ m_receive_write_pointer  ] = character;
  ++m_receive_write_pointer;
  m_receive_write_pointer %= m_receive_buffer_size;
}

uint8_t uart_dpi::deque_receive_byte ( void )
{
  assert( !is_receive_buffer_empty() );
  
  const uint8_t b = m_receive_buffer[ m_receive_read_pointer ];
  ++m_receive_read_pointer;
  m_receive_read_pointer %= m_receive_buffer_size;
  return b;
}

int uart_dpi::get_received_byte_count ( void )
{
  if ( m_receive_read_pointer <= m_receive_write_pointer )
  {
    // any space between the first read and
    // the first write is available.  In this case i
    // is all in one piece.
    return m_receive_write_pointer - m_receive_read_pointer;
  }

  const int ret = m_receive_buffer_size - ( m_receive_read_pointer - m_receive_write_pointer );
  assert( ret > 0 );
  return ret;
}


bool uart_dpi::is_transmit_buffer_full ( void )
{
    return ((m_transmit_write_pointer + 1) % m_transmit_buffer_size) == m_transmit_read_pointer;
}

bool uart_dpi::is_transmit_buffer_empty ( void )
{
    return m_transmit_read_pointer == m_transmit_write_pointer;
}

uint8_t uart_dpi::deque_transmit_byte ( void )
{
  assert( !is_transmit_buffer_empty() );
  
  const uint8_t b = m_transmit_buffer[ m_transmit_read_pointer ];
  ++m_transmit_read_pointer;
  m_transmit_read_pointer %= m_transmit_buffer_size;
  return b;
}


uart_dpi::uart_dpi ( const int tcp_port,
                     const unsigned char listen_on_local_addr_only,
                     const int transmit_buffer_size,
                     const int receive_buffer_size,
                     const char * const welcome_message,
                     const unsigned char print_informational_messages,
                     const char * const informational_message_prefix)
{
  m_listening_socket = -1;
  m_listening_message_already_printed = false;
  m_connectionSocket = -1;
  m_transmit_buffer = NULL;
  m_receive_buffer  = NULL;
  
  if ( tcp_port == 0 )
  {
    throw std::runtime_error( "Invalid TCP port." );
  }

  m_welcome_message = welcome_message ? welcome_message : "";
    
  m_listening_tcp_port = tcp_port;

    
  switch ( print_informational_messages )
  {
  case 0:
    m_print_informational_messages = false;
    break;
      
  case 1:
    m_print_informational_messages = true;
    break;

  default:
    throw std::runtime_error( "Invalid print_informational_messages parameter." );
  }

  m_informational_message_prefix = informational_message_prefix ? informational_message_prefix : "";
    
  switch ( listen_on_local_addr_only )
  {
  case 0:
    m_listen_on_local_addr_only = false;
    break;

  case 1:
    m_listen_on_local_addr_only = true;
    break;

  default:
    throw std::runtime_error( "Invalid listen_on_local_addr_only parameter." );
  }


  // The UART FIFO have 16 bytes and the UART trigger level flags range
  // from 1 to 14, so it's safer to assume the buffer size here is at least 16 bytes.
  const int MIN_BUFFER_SIZE = 16;
    
  if ( receive_buffer_size < MIN_BUFFER_SIZE )
    throw std::runtime_error( "Invalid receive buffer size." );

  m_receive_buffer_size = receive_buffer_size + 1;  // One slot remains unused.
      
  if ( transmit_buffer_size < MIN_BUFFER_SIZE )
    throw std::runtime_error( "Invalid transmit buffer size." );

  m_transmit_buffer_size = transmit_buffer_size + 1;  // One slot remains unused.

  m_receive_buffer = (uint8_t *) malloc( m_receive_buffer_size );
  if ( m_receive_buffer == NULL )
    throw std::runtime_error( "Out of memory allocating the receive buffer." );
    
  m_transmit_buffer = (uint8_t *) malloc( m_transmit_buffer_size );
  if ( m_transmit_buffer == NULL )
    throw std::runtime_error( "Out of memory allocating the transmit buffer." );

  m_receive_write_pointer  = 0;
  m_receive_read_pointer   = 0;
  m_transmit_write_pointer = 0;
  m_transmit_read_pointer  = 0;
  
  create_listening_socket();
}


uart_dpi::~uart_dpi ( void )
{
  if ( m_listening_socket != -1 )
  {
    close_listening_socket();
  }

  if ( m_connectionSocket != -1 )
  {
    close_current_connection();
  }

  free( m_transmit_buffer );
      
  free( m_receive_buffer );
}


void uart_dpi::transmit_data ( void )
{
  // POSSIBLE OPTIMISATION: We could send a block of bytes at once, not just a single byte at a time.

  while ( !is_transmit_buffer_empty() || m_welcome_message_pos != -1 )
  {
    pollfd polledFd;

    polledFd.fd      = m_connectionSocket;
    polledFd.events  = POLLOUT | POLLERR;
    polledFd.revents = 0;

    const int pollRes = poll( &polledFd, 1, 0 );

    if ( pollRes == -1 )
    {
      throw std::runtime_error( get_error_message( "Error polling the connection socket: ", errno ) );
    }

    if ( pollRes == 0 )
      break;

    assert( pollRes == 1 );

    uint8_t byte_to_send;

    if ( m_welcome_message_pos != -1 )
    {
      byte_to_send = m_welcome_message[ m_welcome_message_pos ];

      ++m_welcome_message_pos;

      if ( m_welcome_message[ m_welcome_message_pos ] == 0 )
        m_welcome_message_pos = -1;
    }
    else
    {
      byte_to_send = deque_transmit_byte();
    }
      
    send_byte( byte_to_send );

    // Simulate an error after sending a fixed number of bytes.
    if ( false )
    {
      static int totalCount = 0;

      if ( ++totalCount == 40 )
      {
        totalCount = 0;
        throw std::runtime_error( "Simulated error." );
      }
    }
  }
}


void uart_dpi::receive_data ( void )
{
  // POSSIBLE OPTIMISATION: We could receive a block of bytes at once, not just a single byte at a time.

  while ( !is_receive_buffer_full() )
  {
    pollfd polledFd;

    polledFd.fd      = m_connectionSocket;
    polledFd.events  = POLLIN | POLLERR;
    polledFd.revents = 0;

    const int pollRes = poll( &polledFd, 1, 0 );

    if ( pollRes == -1 )
    {
      throw std::runtime_error( get_error_message( "Error polling the connection socket: ", errno ) );
    }

    if ( pollRes == 0 )
      return;

    assert( pollRes == 1 );

    uint8_t received_data;

    const ssize_t received_byte_count = recv( m_connectionSocket,
                                              &received_data,
                                              1, // Receive just 1 byte.
                                              0  // No special flags.
                                              );
    if ( received_byte_count == 0 )
    {
      if ( m_print_informational_messages )
      {
        printf( "%sConnection closed at the other end.\n", m_informational_message_prefix.c_str() );
        fflush( stdout );
      }
      close_current_connection();
      return;
    }
    
    if ( received_byte_count == -1 )
    {
      if ( errno == EAGAIN || errno == EWOULDBLOCK )
      {
        // No data available yet.
        return;
      }

      throw std::runtime_error( get_error_message( "Error receiving data: ", errno ) );
    }

    assert( received_byte_count == 1 );

    // printf( "Received char: %c\n", received_data );
          
    enqueue_receive_byte( received_data );
  }
}


void uart_dpi::tick ( int * const received_byte_count )
{
    accept_eventual_incoming_connection();
    
    if ( m_connectionSocket != -1 )
    {
      try
      {
        transmit_data();
        receive_data();
      }
      catch ( const std::exception & e )
      {
        fprintf( stderr,
                 "%sConnection closed after error: %s\n",
                 ERROR_MSG_PREFIX,
                 e.what() );
        fflush( stderr );
    
        // Close the connection. The remote client can reconnect later.
        close_current_connection();
      }
    }

    *received_byte_count = get_received_byte_count();
}


void uart_dpi::send_char ( const char character )
{
  if ( is_transmit_buffer_full() )
  {
    deque_transmit_byte();
    assert( ! is_transmit_buffer_full() );
  }

  m_transmit_buffer[ m_transmit_write_pointer  ] = character;
  ++m_transmit_write_pointer;
  m_transmit_write_pointer %= m_transmit_buffer_size;
}


char uart_dpi::receive ( void )
{
  if ( is_receive_buffer_empty() )
  {
    throw std::runtime_error( "The receive buffer is empty." );
  }

  return deque_receive_byte();
}


// ---------------------------- DPI interface ----------------------------

int uart_dpi_create ( const int tcp_port,
                      const unsigned char listen_on_local_addr_only,
                      const int transmit_buffer_size,
                      const int receive_buffer_size,
                      const char * const welcome_message,
                      const unsigned char print_informational_messages,
                      const char * const informational_message_prefix,
                      long long * const obj )
{
  *obj = 0;  // In case of error, return the equivalent of NULL.
             // Otherwise, the 'final' Verilog section must check whether uart_dpi_create() failed before calling uart_dpi_destroy().

  uart_dpi * this_obj = NULL;
  
  try
  {
    this_obj = new uart_dpi( tcp_port,
                             listen_on_local_addr_only,
                             transmit_buffer_size,
                             receive_buffer_size,
                             welcome_message,
                             print_informational_messages,
                             informational_message_prefix );

    // Here there was something else in the past, that's the reason
    // behind the delete in the catch section.
  }
  catch ( const std::exception & e )
  {
    // We should return this error string to the caller,
    // but Verilog does not have good support for variable-length strings.
    fprintf( stderr, "%s%s\n", ERROR_MSG_PREFIX, e.what() );
    fflush( stderr );

    delete this_obj;
    
    return RET_FAILURE;
  }
  catch ( ... )
  {
    fprintf( stderr, "%sUnexpected C++ exception.\n", ERROR_MSG_PREFIX );
    fflush( stderr );

    delete this_obj;
    
    return RET_FAILURE;
  }

  assert( sizeof(*obj) >= sizeof(this_obj) );
  *obj = (long long)this_obj;
  return RET_SUCCESS;
}


void uart_dpi_destroy ( const long long obj )
{
  const uart_dpi * const this_obj = (uart_dpi *)obj;
  
  delete this_obj;
}


int uart_dpi_send ( const long long obj, const char character )
{
  try
  {
    uart_dpi * const this_obj = (uart_dpi *)obj;

    this_obj->send_char( character );
  }
  catch ( const std::exception & e )
  {
    fprintf( stderr, "%s%s\n", ERROR_MSG_PREFIX, e.what() );
    fflush( stderr );
    return RET_FAILURE;
  }
  catch ( ... )
  {
    fprintf( stderr, "%sUnexpected C++ exception.\n", ERROR_MSG_PREFIX );
    fflush( stderr );
    return RET_FAILURE;
  }
  
  return RET_SUCCESS;
}


int uart_dpi_receive ( const long long obj, char * const character )
{
  try
  {
    uart_dpi * const this_obj = (uart_dpi *)obj;

    *character = this_obj->receive();
  }
  catch ( const std::exception & e )
  {
    fprintf( stderr, "%s%s\n", ERROR_MSG_PREFIX, e.what() );
    fflush( stderr );
    return RET_FAILURE;
  }
  catch ( ... )
  {
    fprintf( stderr, "%sUnexpected C++ exception.\n", ERROR_MSG_PREFIX );
    fflush( stderr );
    return RET_FAILURE;
  }
  
  return RET_SUCCESS;
}


int uart_dpi_tick ( const long long obj,
                    int * const received_byte_count )
{
  try
  {
    uart_dpi * const this_obj = (uart_dpi *)obj;

    this_obj->tick( received_byte_count );
  }
  catch ( const std::exception & e )
  {
    fprintf( stderr, "%s%s\n", ERROR_MSG_PREFIX, e.what() );
    fflush( stderr );
    return RET_FAILURE;
  }
  catch ( ... )
  {
    fprintf( stderr, "%sUnexpected C++ exception.\n", ERROR_MSG_PREFIX );
    fflush( stderr );
    return RET_FAILURE;
  }
  
  return RET_SUCCESS;
}
