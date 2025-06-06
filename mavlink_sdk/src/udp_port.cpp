/****************************************************************************
 *
 *   Copyright (c) 2018 MAVlink Development Team. All rights reserved.
 *   Author: Hannes Diethelm, <hannes.diethelm@gmail.com>
 *           Trent Lukaczyk, <aerialhedgehog@gmail.com>
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
 * @file udp_port.cpp
 *
 * @brief UDP interface functions
 *
 * Functions for opening, closing, reading and writing via UDP ports
 *
 * @author Hannes Diethelm, <hannes.diethelm@gmail.com>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <iostream>
#include <chrono>
#include <thread>


#include "udp_port.h"


// ----------------------------------------------------------------------------------
//   UDP Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
mavlinksdk::comm::UDPPort::UDPPort(const char * target_ip_, int udp_port_)
{
	initialize_defaults();
	target_ip_cached = std::string(target_ip_);
	target_ip = target_ip_cached.c_str();
	rx_port  = udp_port_;
	is_open = false;
}

mavlinksdk::comm::UDPPort::
UDPPort()
{
	initialize_defaults();
}

mavlinksdk::comm::UDPPort::
~UDPPort()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void
mavlinksdk::comm::UDPPort::
initialize_defaults()
{
	// Initialize attributes
	target_ip = "127.0.0.1";
	rx_port  = 16455;
	tx_port  = -1;
	is_open = false;
	debug = false;
	m_SocketFD = -1;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from UDP
// ------------------------------------------------------------------------------
int mavlinksdk::comm::UDPPort::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = 0;

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
		msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_UDP, cp, &message, &status);

		// check for dropped packets
		if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=cp;
			fprintf(stderr,"%02x \n", v);
		}
		lastStatus = status;
	}

	// Couldn't read from port
	else
	{
		fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n", result, errno);
	}

	// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		printf("Received message from UDP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr,"Received UDP data: ");
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
	return (msgReceived!=0);
}

// ------------------------------------------------------------------------------
//   Write to UDP
// ------------------------------------------------------------------------------
int mavlinksdk::comm::UDPPort::write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	if (len >= 300) 
	{
		std::cout << "ERROR LEN = " << std::to_string(len) << std::endl;
		exit (0);
	}

	// Write buffer to UDP port, locks port while writing
	int bytesWritten = _write_port(buf,len);
	if(bytesWritten < 0){
		fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n", bytesWritten, errno);
	}

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open UDP Port
// ------------------------------------------------------------------------------

/**
 * throws EXIT_FAILURE if could not open the port
 */
void mavlinksdk::comm::UDPPort::start()
{
	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------

	/* Create socket */
	m_SocketFD = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_SocketFD < 0)
	{
		perror("error socket failed");
		throw EXIT_FAILURE;
	}

	/* Bind the socket to rx_port - necessary to receive packets */
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(target_ip);
	addr.sin_port = htons(rx_port);

	if (bind(m_SocketFD, (struct sockaddr *) &addr, sizeof(struct sockaddr)))
	{
		perror("error bind failed");
		close(m_SocketFD);
		m_SocketFD = -1;
		throw EXIT_FAILURE;
	}

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	//printf("Listening to %s:%i\n", target_ip, rx_port);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return;

}


// ------------------------------------------------------------------------------
//   Close UDP Port
// ------------------------------------------------------------------------------
void mavlinksdk::comm::UDPPort::stop()
{
	std::cout << _INFO_CONSOLE_TEXT << "Closing UDP Port" << _NORMAL_CONSOLE_TEXT_ << std::endl;    

	int result = close(m_SocketFD);
	m_SocketFD = -1;

	if ( result )
	{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR: Error on port close (" << result << ")" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
	}

	is_open = false;

	return;
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int mavlinksdk::comm::UDPPort::_read_port(uint8_t &cp)
{

	socklen_t sender_address_size;

	// Lock
	pthread_mutex_lock(&lock);

	int result = -1;
	if(buff_ptr < buff_len){
		cp=buff[buff_ptr];
		buff_ptr++;
		result=1;
	}else{
		struct timeval tv;
    	tv.tv_sec = 1; // timeout_in_seconds;
    	tv.tv_usec = 0;
    	setsockopt(m_SocketFD, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

		struct sockaddr_in addr;
		sender_address_size = sizeof(struct sockaddr_in);
		result = recvfrom(m_SocketFD, (char *)buff, BUFF_LEN,  
                MSG_WAITALL, ( struct sockaddr *) &addr, &sender_address_size);
        
        if (result==-1)	 
		{
			pthread_mutex_unlock(&lock);
			//std::this_thread::sleep_for(500);
			std::this_thread::sleep_for(std::chrono::milliseconds(200));

			return result;
		}
		
		//always read port as ardupilot app may restart and get another port.
		if(strcmp(inet_ntoa(addr.sin_addr), target_ip) == 0){
			tx_port = ntohs(addr.sin_port);
			//printf("Got first packet, sending to %s:%i\n", target_ip, rx_port);
		}else{
			target_ip = inet_ntoa(addr.sin_addr);
			printf("ERROR: Got packet from %s:%i but listening on %s\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port), target_ip);
		}

		if(result > 0){
			buff_len=result;
			buff_ptr=0;
			cp=buff[buff_ptr];
			buff_ptr++;
		}
	}

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int
mavlinksdk::comm::UDPPort::
_write_port(char *buf, unsigned len)
{

	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via UDP link
	int bytesWritten = 0;
	if(tx_port > 0){
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr(target_ip);
		addr.sin_port = htons(tx_port);
		bytesWritten = sendto(m_SocketFD, buf, len, 0, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
	}else{
		std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR: Sending before first packet received!" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
		bytesWritten = -1;
	}

	// Unlock
	pthread_mutex_unlock(&lock);


	return bytesWritten;
}


