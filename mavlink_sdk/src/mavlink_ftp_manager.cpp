#include <iostream>


#include "./helpers/colors.h"
#include "mavlink_helper.h"
#include "mavlink_command.h"
#include "mavlink_sdk.h"
#include "mavlink_ftp_manager.h"



void mavlinksdk::CMavlinkFTPManager::requestMavFTPParamList()
{
    std::cout << __FILE__ << "." << __FUNCTION__ <<  " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG:requestMavFTPParamList"  << _NORMAL_CONSOLE_TEXT_ << std::endl;

    mavlink_file_transfer_protocol_t mavlink_ftp;

	mavlinksdk::CMavlinkSDK& mavlink_sdk = mavlinksdk::CMavlinkSDK::getInstance();
	
	mavlink_ftp.target_system = mavlink_sdk.getSysId();
	mavlink_ftp.target_component = mavlink_sdk.getCompId();
	mavlink_ftp.target_network = 0;

	

	
	
	// Encode
	mavlink_message_t mavlink_message;
	mavlink_msg_file_transfer_protocol_encode(mavlink_sdk.getSysId(), mavlink_sdk.getCompId(), &mavlink_message, &mavlink_ftp);

    mavlink_sdk.sendMavlinkMessage(mavlink_message);

	return ;
}