#include "helpers/helpers.hpp"
#include "fcb_traffic_optimizer.hpp"


void uavos::fcb::CMavlinkTrafficOptimizer::init(const Json &mavlink_messages_config)
{
    int n=mavlink_messages_config.size(); // n = 2
    for(auto it=mavlink_messages_config.begin();it!=mavlink_messages_config.end();++it){
        //std::cout << it.key() << std::endl;
        int message_id = std::stoi (it.key());
        const std::vector<int> values = it.value();
        int index = 0;
        for (auto it = values.begin(); it != values.end(); ++it){
            //std::cout << *it << std::endl;
            m_message[message_id].timeout[index%4] = *it;
            index++;
        }
    }
}

bool uavos::fcb::CMavlinkTrafficOptimizer::shouldForwardThisMessage (const mavlink_message_t& mavlink_message)
{
    const std::uint64_t now = get_time_usec();
    T_MessageOptimizeCard *message_optimization_card = &m_message[mavlink_message.msgid];
    if ((now - message_optimization_card->time_of_last_sent_message) >= message_optimization_card->timeout[m_optimization_level])
    {
        message_optimization_card->time_of_last_sent_message = now;
        return true;
    }
    return false;
}