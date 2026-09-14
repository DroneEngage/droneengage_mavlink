#include "./de_common/helpers/helpers.hpp"
#include "./de_common/helpers/colors.hpp"
#include "fcb_traffic_optimizer.hpp"


using namespace de::fcb;


void CMavlinkTrafficOptimizer::init(const Json_de &mavlink_messages_config)
{
    std::lock_guard<std::mutex> lock(m_lock);
    for(auto it=mavlink_messages_config.begin();it!=mavlink_messages_config.end();++it){
        //std::cout << it.key() << std::endl;
        try
        {
            int message_id = std::stoi (it.key());
            const std::vector<int> values = it.value();
            T_MessageOptimizeCard card{};

            if (!values.empty())
            {
                const std::uint64_t last_timeout_usec = static_cast<std::uint64_t>(std::max(values.back(), 0)) * 1000ULL;
                for (int i = 0; i < OPTIMIZE_LEVELS; ++i)
                {
                    const std::uint64_t timeout_usec = (i < static_cast<int>(values.size())) ? (static_cast<std::uint64_t>(std::max(values[i], 0)) * 1000ULL) : last_timeout_usec;
                    card.timeout[i] = timeout_usec;
                }
            }
            m_message.insert(std::make_pair(message_id,card));
        }
        catch (const std::exception& e)
        {
            std::cout << _ERROR_CONSOLE_TEXT_ << "message_timeouts[" << it.key() << "] invalid, skipping: " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
    }

    std::cout << _INFO_CONSOLE_TEXT << "Loaded " << m_message.size() << " traffic-optimization rules" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}

bool CMavlinkTrafficOptimizer::shouldForwardThisMessage (const mavlink_message_t& mavlink_message)
{
    std::lock_guard<std::mutex> lock(m_lock);
    const std::uint64_t now = get_time_usec_monotonic();
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