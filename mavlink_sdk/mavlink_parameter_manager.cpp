#include <iostream>
#include "./helpers/colors.h"
#include "./helpers/utils.h"
#include "mavlink_helper.h"
#include "mavlink_command.h"
#include "mavlink_parameter_manager.h"





using namespace mavlinksdk;


/**
 * @brief callback class of time 
 * 
 * @param callback_parameter mavlinksdk::CCallBack_Parameter
 */
void mavlinksdk::CMavlinkParameterManager::set_callback_parameter (mavlinksdk::CCallBack_Parameter* callback_parameter)
{
    m_callback_parameter = callback_parameter;
}


/**
 * @brief initialize process of calling paramaters from FCB. 
 * @details it sets {@link m_parameter_read_mode} to LOADING_PARAMS_LOAD_ALL_INIT
 */
void mavlinksdk::CMavlinkParameterManager::reloadParemeters ()
{
    m_parameter_read_mode = mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LOAD_ALL_INIT;

    mavlinksdk::CMavlinkCommand::getInstance().requestParametersList();
}



/**
 * @brief 
 * List of all parameters as a response of PARAM_REQUEST_LIST
 * 
 * @param param_message 
 */
void mavlinksdk::CMavlinkParameterManager::handle_param_value (const mavlink_param_value_t& param_message)
{
	bool changed = false;
	
	char param_id[17];
	param_id[16] =0;
	memcpy((void *)&param_id[0], param_message.param_id,16);
	std::string param_name = std::string(param_id);

	auto it = m_parameters_list.find(param_name);

	// search for parameter in the received list and check if undapted or not.
	if (it != m_parameters_list.end()) 
	{
		changed = (it->second.param_value != param_message.param_value);
		if (changed) 
		{
			it->second.param_value = param_message.param_value;
			//IMPORTANT: param_index is 65535 when value is re-read.
		}
	} 
	
	// if param_index is value then it is a new parameter.
	if (param_message.param_index < param_message.param_count)
	{ 
		// fresh account with index valid.
		#ifdef DEBUG
		std::cout << "Fresh m_parameters_last_index_read: " << std::to_string(param_message.param_index) << std::endl;
		#endif 
		
		m_parameters_last_receive_time =  get_time_usec();
		m_parameter_read_count = param_message.param_count;
		m_parameters_last_index_read = param_message.param_index;
		m_parameters_list.insert(std::make_pair(param_name, param_message));

		bool bFound = false;
		for(auto itr=m_parameters_id.begin(); itr !=m_parameters_id.end(); itr++)
		{
        	if (*itr == param_message.param_index)
			{
				bFound = true;
				break;
			}
		}

		if (!bFound)
		{
			m_parameters_id.push_back(param_message.param_index);
		}
	}

	// if last parameter has been received the this is the last parameter
	if (param_message.param_index == (param_message.param_count-1)) 
	{ 
		m_parameter_read_mode = mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_LOADED;
		std::cout << _SUCCESS_CONSOLE_TEXT_ << "Parameter LOADED " << _NORMAL_CONSOLE_TEXT_ << std::endl;
		mavlinksdk::CMavlinkCommand::getInstance().requestDataStream();
		m_callback_parameter->OnParamReceivedCompleted();
	}

	m_callback_parameter->OnParamReceived (param_name, param_message, changed);

	if (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_ONE_BY_ONE)
	{
		if (m_parameters_last_index_read < m_parameter_read_count)
		{
			// request next parameter index
			mavlinksdk::CMavlinkCommand::getInstance().readParameterByIndex(m_parameters_last_index_read+1);
		}
	}
 	

	#ifdef DEBUG
	std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: " 
		<< param_name << " : " << "count: " << std::to_string(param_message.param_index) << " of " << std::to_string(param_message.param_count)
		<< " type: " << std::to_string(param_message.param_type) << " value: " << std::to_string(param_message.param_value)
		<< _NORMAL_CONSOLE_TEXT_ << std::endl;
	#endif
}


/**
 * @brief parameters are loaded.
 * @details first time a request to load all parameters is reloadParemeters() then if timeout and not all
 * parameters are received then a one-by-one call is executed untill all parameters are received. m_parameter_read_mode is used to store status.
 * @callgraph
 * @param heartbeat mavlink message
 */
void mavlinksdk::CMavlinkParameterManager::handle_heart_beat (const mavlink_heartbeat_t& heartbeat)
{
    const uint64_t now = get_time_usec();

    #ifdef DEBUG
		std::cout << "Timeout readings: " << std::to_string((now - m_parameters_last_receive_time)) << std::endl;
	#endif 
	// Initialize request all poarameters
	if (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_EMPTY)
	{
		reloadParemeters();

		return ;
	}

	// no parameter has been received yet.
	if ((m_parameters_last_receive_time==0) 
	&& (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LOAD_ALL_INIT) // redundant condition flor clarity only.
	&& ((now - m_parameters_last_receive_time) >  300000l))
	{
		// Load all parameters at first
		reloadParemeters();

		return ;
	}
		

	// If still loading & timeout then load one-by-one.
	if ((m_parameter_read_mode != mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_LOADED)  
		 && ((now - m_parameters_last_receive_time) >  300000l))
	{
		if (m_parameters_last_receive_time==0)
		{
			// Load all parameters at first
			reloadParemeters();
		}
		else
		{
			#ifdef DEBUG
				std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  
				<< _LOG_CONSOLE_TEXT << "DEBUG: MISSING PARAMs m_parameters_last_index_read:" << std::to_string(m_parameters_last_index_read) 
				<< std::endl;
			#endif
			m_parameter_read_mode = mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_ONE_BY_ONE;
			
			const uint16_t index = getFirstMissingParameterByIndex();
			if (index<m_parameter_read_count) 
			{
				mavlinksdk::CMavlinkCommand::getInstance().readParameterByIndex(index);
			}
		}
	}
}

uint16_t mavlinksdk::CMavlinkParameterManager::getFirstMissingParameterByIndex()
{
	bool bFound = false;
	uint16_t index = 0;
	for (int i=0; i< m_parameter_read_count; ++i)
	{
		bFound = false;
		for(auto itr=m_parameters_id.begin(); itr !=m_parameters_id.end(); itr++)
		{
			if (i==*itr)
			{
				bFound = true;
				break;
			}
		}
		
		if (!bFound) 
		{
			std::cout<<"\n index:"<<index<<std::endl;
			return i;
		}
	}

	
	return m_parameter_read_count;
}
