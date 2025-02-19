
#include "../helpers/colors.hpp"
#include "de_facade_base.hpp"




using namespace de::comm;



/**
 * @brief ORIGINAL
 * 
 * @param target_party_id 
 */
void CFacade_Base::requestID(const std::string&target_party_id) const
{
    
    Json_de message = 
        {
            {"C", TYPE_AndruavMessage_ID}
        };
        

    m_module.sendJMSG (target_party_id, message, TYPE_AndruavMessage_RemoteExecute, true);
    
    return ;
}


void CFacade_Base::sendErrorMessage (const std::string&target_party_id, const int& error_number, const int& info_type, const int& notification_type, const std::string& description)  const
{
    /*
        EN : error number  "not currently processed".
        IT : info type indicate what component is reporting the error.
        NT : sevirity and com,pliant with ardupilot.
        DS : description message.
    */
    Json_de message =
        {
            {"EN", error_number},
            {"IT", info_type},
            {"NT", notification_type},
            {"DS", description}
        };

    m_module.sendJMSG (target_party_id, message, TYPE_AndruavMessage_Error, false);
    
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << " -- sendErrorMessage " << _NORMAL_CONSOLE_TEXT_ << description << std::endl;
    
    return ;
}