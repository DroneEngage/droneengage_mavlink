/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file serial_port.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <sstream>
#include "serial_port.h"



// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
mavlinksdk::comm::SerialPort::
SerialPort(const char *uart_name , int baudrate, const bool dynamic)
{
	initialize_defaults();
	_uart_name = std::string(uart_name);
	_baudrate  = baudrate;
	_dynamic = dynamic;
}

mavlinksdk::comm::SerialPort::
SerialPort()
{
	initialize_defaults();
}

mavlinksdk::comm::SerialPort::
~SerialPort()
{
	// destroy mutex
	pthread_mutex_destroy(&lockw);
	pthread_mutex_destroy(&lockr);
}

void
mavlinksdk::comm::SerialPort::
initialize_defaults()
{
	// Initialize attributes
	debug  = false;
	fd     = -1;
	_is_open = false;

	_uart_name = std::string("/dev/ttyUSB0");
	_baudrate  = 57600;

	// Start mutex
	int result = pthread_mutex_init(&lockr, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
	result = pthread_mutex_init(&lockw, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int mavlinksdk::comm::SerialPort::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	int result = _read_port(cp);


	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if (result > 0)
	{
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_SERIAL, cp, &message, &status);

		// check for dropped packets
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=cp;
			fprintf(stderr,"%02x ", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		std::cout << _ERROR_CONSOLE_TEXT_  << "ERROR: Could not read from serial port" << _ERROR_CONSOLE_TEXT_ << std::endl;
		
		_try_reopen();
	}

	// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
		_got_mavlink = true;
		
		fprintf(stderr,"Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}

		// print out the buffer
		else
		{
			for (i=0; i<messageLength; i++)
			{
				unsigned char v=buffer[i];
				fprintf(stderr,"%02x ", v);
			}
			fprintf(stderr,"\n");
		}
	}

	// Done!
	return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
mavlinksdk::comm::SerialPort::
write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = _write_port(buf,len);

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
mavlinksdk::comm::SerialPort::
start()
{

	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
		
	_got_mavlink = false;
	
	std::ostringstream uart_name;
	
	uart_name << _uart_name;

	if (_dynamic)
	{
		_portext++;
		_portext = _portext % 10;
		
		uart_name << std::to_string(_portext);
		
	}
	std::cout << _INFO_CONSOLE_TEXT  << " Try to open serial port: " << _SUCCESS_CONSOLE_TEXT_ << uart_name.str()  << _NORMAL_CONSOLE_TEXT_ << std::endl;
	
	fd = _open_port(uart_name.str().c_str());

	// Check success
	if (fd == -1)
	{
		_is_open = true;
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_  << "Failure," << _ERROR_CONSOLE_TEXT_ << " could not configure port: "  << _INFO_CONSOLE_TEXT << uart_name.str() << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	bool success = _setup_port(_baudrate, 8, 1, false, false);

	// --------------------------------------------------------------------------
	//   CHECK STATUS
	// --------------------------------------------------------------------------
	if (!success)
	{
		std::cout << _ERROR_CONSOLE_TEXT_  << "failure," << _ERROR_CONSOLE_TEXT_ << " could not configure port." << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return ;
	}
	

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_  << "SUCCESS: " << _SUCCESS_CONSOLE_TEXT_ << "Connection attempt to port " << _INFO_CONSOLE_TEXT <<  uart_name.str() << _SUCCESS_CONSOLE_BOLD_TEXT_ << " with "<< _baudrate << " baud, 8 data bits, no parity, 1 stop bit (8N1)." << _NORMAL_CONSOLE_TEXT_ << std::endl;
	lastStatus.packet_rx_drop_count = 0;

	_is_open = true;

	return;
}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void mavlinksdk::comm::SerialPort::stop()
{
	_reOpen = false;
	closePort();
}

void mavlinksdk::comm::SerialPort::closePort ()
{
	std::cout << _INFO_CONSOLE_TEXT << "Closing Serial Port" << _NORMAL_CONSOLE_TEXT_ << std::endl;    

	int result = close(fd);

	if ( result )
	{
		std::cout << _INFO_CONSOLE_TEXT << "WARNING: Error on port close (" << result << ")" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
	}

	_is_open = false;

	printf("\n");
}

bool mavlinksdk::comm::SerialPort::_try_reopen()
{
	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	
	if (!_reOpen) return false;
	std::ostringstream uart_name;
	
	closePort();

	uart_name << _uart_name;

	if (_dynamic)
	{
		_portext++;
		_portext = _portext % 10;
		
		uart_name << std::to_string(_portext);
		
	}
	std::cout << _INFO_CONSOLE_TEXT  << " Try to open serial port: " << _SUCCESS_CONSOLE_TEXT_ << uart_name.str()  << _NORMAL_CONSOLE_TEXT_ << std::endl;
	fd = _open_port(uart_name.str().c_str());


	// Check success
	if (fd == -1)
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_  << "failure, could not configure port: "  << _INFO_CONSOLE_TEXT << uart_name.str() << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return false; // return here to try again.
	}

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	bool success = _setup_port(_baudrate, 8, 1, false, false);

	// --------------------------------------------------------------------------
	//   CHECK STATUS
	// --------------------------------------------------------------------------
	if (!success)
	{
		std::cout << _ERROR_CONSOLE_TEXT_  << "failure, could not configure port." << _NORMAL_CONSOLE_TEXT_ << std::endl;
		return false;
	}
	

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_  << "SUCCESS: " << _SUCCESS_CONSOLE_TEXT_ << "Connection attempt to port " << _INFO_CONSOLE_TEXT <<  uart_name.str() << _SUCCESS_CONSOLE_BOLD_TEXT_ << " with "<< _baudrate << " baud, 8 data bits, no parity, 1 stop bit (8N1)." << _NORMAL_CONSOLE_TEXT_ << std::endl;
	lastStatus.packet_rx_drop_count = 0;

	_is_open = true;
	return true;
}

// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int mavlinksdk::comm::SerialPort::_open_port(const char* port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}

	// Finalize
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

	// Done!
	return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool
mavlinksdk::comm::SerialPort::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	// Check file descriptor
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 500000:
			if (cfsetispeed(&config, B500000) < 0 || cfsetospeed(&config, B500000) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1500000:
			if (cfsetispeed(&config, B1500000) < 0 || cfsetospeed(&config, B1500000) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	// Finally, apply the configuration
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}

	// Done!
	return true;
}



// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int
mavlinksdk::comm::SerialPort::
_read_port(uint8_t &cp)
{

	if (fd == -1) return 0;
	
	// Lock
	pthread_mutex_lock(&lockr);
	try
	{
	int rv;
	fd_set set;
	struct timeval timeout = {1, 0};
	FD_ZERO(&set); /* clear the set */
  	FD_SET(fd, &set); /* add our file descriptor to the set */
	rv = select(fd + 1, &set, NULL, NULL, &timeout);
	if(rv == -1)
		perror("select"); /* an error accured */
	else if(rv == 0)
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "Error: Serial Read timout. Maytry another port" << _ERROR_CONSOLE_TEXT_ << std::endl;
	else
	{
		int result = read(fd, &cp, 1);
		pthread_mutex_unlock(&lockr);
		return result;
	}
	}
	catch (int error)
	{
		pthread_mutex_unlock(&lockr);
		return 0;
	}
	// Unlock
	pthread_mutex_unlock(&lockr);

	return 0;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int
mavlinksdk::comm::SerialPort::
_write_port(char *buf, unsigned len)
{

	// Lock
	pthread_mutex_lock(&lockw);

	// Write packet via serial link
	const int bytesWritten = static_cast<int>(write(fd, buf, len));

	// Wait until all data has been written
	tcdrain(fd);

	// Unlock
	pthread_mutex_unlock(&lockw);


	return bytesWritten;
}


