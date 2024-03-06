#include "helpers/helpers.hpp"
#include "fcb_traffic_optimizer.hpp"


void uavos::fcb::CMavlinkTrafficOptimizer::init(const Json_de &mavlink_messages_config)
{
    for(auto it=mavlink_messages_config.begin();it!=mavlink_messages_config.end();++it){
        //std::cout << it.key() << std::endl;
        int message_id = std::stoi (it.key());
        const std::vector<int> values = it.value();
        int index = 0;
        T_MessageOptimizeCard card;
        
        for (auto it = values.begin(); it != values.end(); ++it){
            //std::cout << *it << std::endl;
            card.timeout[index%4] = *it * 1000;
            index++;
        }
        m_message.insert(std::make_pair(message_id,card));
    }
}

bool uavos::fcb::CMavlinkTrafficOptimizer::shouldForwardThisMessage (const mavlink_message_t& mavlink_message)
{
    const std::uint64_t now = get_time_usec();
    auto it = m_message.find(mavlink_message.msgid);
    if (it != m_message.end())
    {
        if ((now - it->second.time_of_last_sent_message) >= it->second.timeout[m_optimization_level])
        {
            it->second.time_of_last_sent_message = now;
            return true;
        }
        else
        {
            return false;
        }
    }   

    return true;
}