#include <iostream>

#include <mutex>
#include <unistd.h>  // UNIX standard function definitions

#include "colors.h"
#include "mavlink_communicator.h"



mavlinksdk::comm::CMavlinkCommunicator::~CMavlinkCommunicator ()
{
    this->stop();
}


void mavlinksdk::comm::CMavlinkCommunicator::start ()
{
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Communicator Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 
    int result;

    result = pthread_create( &m_read_tid, NULL, &mavlinksdk::comm::startReadThread, this );
	if ( result ) throw result;

    result = pthread_create( &m_read_tid, NULL, &mavlinksdk::comm::startWriteThread, this );
	if ( result ) throw result;
    
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
	pthread_join(m_read_tid ,NULL);
	pthread_join(m_read_tid ,NULL);

	// now the read and write threads are closed
	std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Communicator has Stopped" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 

	// still need to close the port separately
}

void mavlinksdk::comm::CMavlinkCommunicator::_readThread ()
{
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "_readThread Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
 
    m_reading_status = true;

	while ( ! m_time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	m_reading_status = false;

	return;
}


void mavlinksdk::comm::CMavlinkCommunicator::_writeThread ()
{
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "_writeThread Started" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
}


void mavlinksdk::comm::CMavlinkCommunicator::read_messages ()
{
    bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	//Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !m_time_to_exit )
	{
        // ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = m_port->read_message(message);

        if( success )
		{
            std::cout << _SUCCESS_CONSOLE_TEXT_ << "Message Received" << _NORMAL_CONSOLE_TEXT_ << std::endl;    
        }
		
    }
}
// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void* mavlinksdk::comm::startReadThread(void *args)
{
	// takes an autopilot object argument
	mavlinksdk::comm::CMavlinkCommunicator *comm = (mavlinksdk::comm::CMavlinkCommunicator *)args;

	// run the object's read thread
	comm->_readThread();

	// done!
	return NULL;
}

void* mavlinksdk::comm::startWriteThread(void *args)
{
	// takes an autopilot object argument
	mavlinksdk::comm::CMavlinkCommunicator *comm = (mavlinksdk::comm::CMavlinkCommunicator *)args;

	// run the object's read thread
	comm->_writeThread();

	// done!
	return NULL;
}