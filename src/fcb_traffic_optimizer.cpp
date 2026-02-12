#include "./de_common/helpers/helpers.hpp"
#include "fcb_traffic_optimizer.hpp"


using namespace de::fcb;


void CMavlinkTrafficOptimizer::init(const Json_de &mavlink_messages_config)
{
    for(auto it=mavlink_messages_config.begin();it!=mavlink_messages_config.end();++it){
        //std::cout << it.key() << std::endl;
        int message_id = std::stoi (it.key());
        const std::vector<int> values = it.value();
        T_MessageOptimizeCard card{};

        if (!values.empty())
        {
            const int last_timeout_usec = values.back() * 1000;
            for (int i = 0; i < OPTIMIZE_LEVELS; ++i)
            {
                const int timeout_usec = (i < static_cast<int>(values.size())) ? (values[i] * 1000) : last_timeout_usec;
                card.timeout[i] = timeout_usec;
            }
        }
        m_message.insert(std::make_pair(message_id,card));
    }
}

bool CMavlinkTrafficOptimizer::shouldForwardThisMessage (const mavlink_message_t& mavlink_message)
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