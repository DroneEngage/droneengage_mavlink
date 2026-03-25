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
void mavlinksdk::CMavlinkParameterManager::reloadParameters ()
{
	m_parameter_read_mode = mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LOAD_ALL_INIT;
	m_start_time = get_time_usec(); // Track when loading started

    mavlinksdk::CMavlinkCommand::getInstance().requestParametersList();
}



/**
 * @brief List of all parameters as a response of PARAM_REQUEST_LIST
 * 
 * @callgraph
 * @param param_message 
 */
void mavlinksdk::CMavlinkParameterManager::handle_param_value (const mavlink_param_value_t& param_message)
{
	bool changed = false;
	
	std::string param_name = std::string(param_message.param_id, 16);
	size_t null_pos = param_name.find('\0');
	if (null_pos != std::string::npos) {
		param_name.erase(null_pos); // Remove null termination
	}

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
		#ifdef DEBUG_DETAILED
		std::cout << "Fresh m_parameters_last_index_read: " << std::to_string(param_message.param_index) << std::endl;
		#endif 
		#endif 
		
		m_parameters_last_receive_time =  get_time_usec();
		m_parameter_read_count = param_message.param_count;
		m_parameters_last_index_read = param_message.param_index;
		m_parameters_list.insert(std::make_pair(param_name, param_message));

		// Use unordered_set for O(1) lookup instead of O(n) vector search
		if (m_parameters_id_set.find(param_message.param_index) == m_parameters_id_set.end())
		{
			m_parameters_id.push_back(param_message.param_index);
			m_parameters_id_set.insert(param_message.param_index);
		}
	}

	m_callback_parameter->OnParamReceived (param_name, param_message, changed, m_load_parameters_1st_iteration);

	// if last parameter has been received then this is the last parameter
	if (param_message.param_index == (param_message.param_count-1)) 
	{ 
		m_parameter_read_mode = mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_LOADED;
		std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "* Parameter LOADED " << _NORMAL_CONSOLE_TEXT_ << std::endl;
		
		m_callback_parameter->OnParamReceivedCompleted();
		
		m_load_parameters_1st_iteration = false;
	}

	if (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_ONE_BY_ONE)
	{
		if (m_parameters_last_index_read < m_parameter_read_count)
		{
			// request next parameter index
			mavlinksdk::CMavlinkCommand::getInstance().readParameterByIndex(m_parameters_last_index_read+1);
		}
	}
 	

	#ifdef DEBUG
	#ifdef DEBUG_DETAILED
	// std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  << _LOG_CONSOLE_TEXT << "DEBUG: " 
	// 	<< param_name << " : " << "count: " << std::to_string(param_message.param_index) << " of " << std::to_string(param_message.param_count)
	// 	<< " type: " << std::to_string(param_message.param_type) << " value: " << std::to_string(param_message.param_value)
	// 	<< _NORMAL_CONSOLE_TEXT_ << std::endl;
	#endif
	#endif
}


/**
 * @brief parameters are loaded.
 * @details first time a request to load all parameters is reloadParameters() then if timeout and not all
 * parameters are received then a one-by-one call is executed untill all parameters are received. m_parameter_read_mode is used to store status.
 * @callgraph
 * @param heartbeat mavlink message
 */
void mavlinksdk::CMavlinkParameterManager::handle_heart_beat (const mavlink_heartbeat_t& heartbeat)
{
    const uint64_t now = get_time_usec();

    #ifdef DEBUG
	#ifdef DEBUG_DETAILED
	 	std::cout << "Timeout readings: " << std::to_string((now - m_parameters_last_receive_time)) << std::endl;
	#endif 
	#endif
	// Initialize request all poarameters
	if (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_EMPTY)
	{
		reloadParameters();

		return ;
	}

	// no parameter has been received yet and timeout occurred since start.
	if ((m_parameters_last_receive_time==0) 
	&& (m_parameter_read_mode == mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LOAD_ALL_INIT) // redundant condition flor clarity only.
	&& (m_start_time > 0) && ((now - m_start_time) >  300000l))
	{
		// Load all parameters at first
		reloadParameters();

		return ;
	}
		

	// If still loading & timeout then load one-by-one.
	if ((m_parameter_read_mode != mavlinksdk::ENUM_LOADING_PARAMS_STATUS::LOADING_PARAMS_LIST_LOADED)  
		 && ((now - m_parameters_last_receive_time) >  300000l))
	{
		if (m_parameters_last_receive_time==0)
		{
			// Load all parameters at first
			reloadParameters();
		}
		else
		{
			#ifdef DEBUG
			#ifdef DEBUG_PARAM
				std::cout <<__FILE__ << "." << __FUNCTION__ << " line:" << __LINE__ << "  "  
				<< _LOG_CONSOLE_TEXT << "DEBUG: MISSING PARAMs m_parameters_last_index_read:" << std::to_string(m_parameters_last_index_read) 
				<< std::endl;
			#endif
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
	for (int i=0; i< m_parameter_read_count; ++i)
	{
		// Use unordered_set for O(1) lookup instead of O(n) vector search
		if (m_parameters_id_set.find(i) == m_parameters_id_set.end())
		{
			std::cout<<"\n missing index:"<<i<<std::endl;
			return i;
		}
	}

	return m_parameter_read_count;
}

/**
 * @brief returns parameter by name. This is a helper function can be used by other modules.
 * 
 * @param pram_name 
 * @callgraph
 * @return const mavlink_param_value_t if mavlink_param_value_t.param_index=-1 then parameter is not found.
 */
const mavlink_param_value_t mavlinksdk::CMavlinkParameterManager::getParameterByName(const std::string param_name) const 
{
	const std::map<std::string, mavlink_param_value_t>& parameters_list = getParametersList();
	
	auto it = parameters_list.find(param_name);

	if (it == parameters_list.end())
	{
		mavlink_param_value_t t;
		t.param_index = -1;
		
		return t; // not found
	} 

	return it->second;
}
            