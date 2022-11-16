#include <iostream>

#include <mutex>
#include <unistd.h>  // UNIX standard function definitions

#include "./helpers/colors.h"
#include "./helpers/utils.h"
#include "mavlink_communicator.h"
#include "mavlink_sdk.h"



mavlinksdk::comm::CMavlinkCommunicator::~CMavlinkCommunicator ()
{
    this->stop();
}


void mavlinksdk::comm::CMavlinkCommunicator::start ()
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Communicator Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 

	m_threadRead = std::thread {[&](){ readThread(); }};

	//m_threadWrite = std::thread {[&](){ writeThread(); }};
    
}

void mavlinksdk::comm::CMavlinkCommunicator::stop ()
{
    // --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Communicator is Stopping Normally" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 
	// signal exit
	m_time_to_exit = true;

	// wait for exit
	
	m_threadRead.join();
	//m_threadWrite.join();

	// now the read and write threads are closed
	std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Communicator has Stopped" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 

	// still need to close the port separately

	return ;
}

const int mavlinksdk::comm::CMavlinkCommunicator::send_message (const mavlink_message_t& mavlink_message)
{
	const int len = m_port->write_message(mavlink_message);

	return len;
}

/**
 * @brief called in a separate thread for loop on reading.
 * @details loop and process message by @link read_messages @endlink 
 */
void mavlinksdk::comm::CMavlinkCommunicator::readThread ()
{
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "_readThread Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 
    m_reading_status = true;

	while ( ! m_time_to_exit )
	{
		read_messages();
		wait_time_nsec(1,0); // Read batches at 10Hz
	}

	m_reading_status = false;

	return;
}


// void mavlinksdk::comm::CMavlinkCommunicator::writeThread ()
// {
//     std::cout << _SUCCESS_CONSOLE_TEXT_ << "_writeThread Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    

// 	while ( !m_time_to_exit )
// 	{
// 		wait_time_nsec(1,0); // Read batches at 10Hz
// 	}
// }


/**
 * @brief does actuall reading.
 * @details reads from port and sends CCallBack_Communicator::OnConnected CCallBack_Communicator::OnMessageRecieved
 * @see @link CCallBack_Communicator @endlink
 */
void mavlinksdk::comm::CMavlinkCommunicator::read_messages ()
{
	// Blocking wait for new data
	while ( !m_time_to_exit )
	{
        // ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		memset(&message,0,sizeof(mavlink_message_t));
		const bool success = m_port->read_message(message);
		
        if( success )
		{
			if (m_connected == false)
			{
				m_connected = true;
				this->m_callback_communicator->OnConnected (true);
			}
            this->m_callback_communicator->OnMessageReceived (message);
        }
    }
}






