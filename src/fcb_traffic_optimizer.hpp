#ifndef FCB_TRAFFIC_OPTIIZER_H_
#define FCB_TRAFFIC_OPTIIZER_H_


#include "./helpers/json.hpp"
using Json = nlohmann::json;

#include <common/mavlink.h>
#include <mavlink_sdk.h>




namespace uavos
{
namespace fcb
{

    #define TIME_STAMP_MSG_LEN 1024
    #define OPTIMIZE_LEVELS     4

    #define OPTIMIZE_LEVEL_0    0   
    #define OPTIMIZE_LEVEL_1    1
    #define OPTIMIZE_LEVEL_2    2
    #define OPTIMIZE_LEVEL_3    3
    #define OPTIMIZATION_LEVEL_DEFAULT OPTIMIZE_LEVEL_2


    typedef struct T_MessageOptimizeCard
    {
        int timeout[OPTIMIZE_LEVELS];
        std::uint64_t time_of_last_sent_message;
    } T_MessageOptimizeCard;


    class CMavlinkTrafficOptimizer
    {

        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CMavlinkTrafficOptimizer& getInstance()
            {
                static CMavlinkTrafficOptimizer instance;

                return instance;
            }

            CMavlinkTrafficOptimizer(CMavlinkTrafficOptimizer const&)               = delete;
            void operator=(CMavlinkTrafficOptimizer const&)                         = delete;

        private:

            CMavlinkTrafficOptimizer()
            {

            }
        
        public:

            ~CMavlinkTrafficOptimizer()
            {

            }

            
        public:

            void init(const Json &mavlink_messages_config);

            /*
            * @details Determines stream optimization. A critical parameter for network bandwidth.
            * @param level from 0 to 3. Zero means no optimization and 3 max optimization.
            */
            void setOptimizationLevel (int level)
            {
                if (level <= OPTIMIZE_LEVEL_0) level = OPTIMIZE_LEVEL_0;
                if (level >= OPTIMIZE_LEVEL_3) level = OPTIMIZE_LEVEL_3;

                m_optimization_level = level;
            }

            int getOptimizationLevel ()
            {
                return m_optimization_level;
            }

            /**
             * @details Returns true if the message should be forward via telemetry to GCS.
             * Settings of this message are stored in config.json file.
             * @link m_optimization_level @endlink is used to select timeout value.
             * @param mavlink_message 
             * @return true 
             * @return false 
             */
            bool shouldForwardThisMessage (const mavlink_message_t& mavlink_message);

            /**
             * @brief Reset time_of_last_sent_message of all messages. 
             * 
             */
            void reset_timestamps()
            {
                for(auto it=m_message.begin();it!=m_message.end();++it)
                {
                    T_MessageOptimizeCard card = it->second;
                    card.time_of_last_sent_message = 0;
                }
                
                return;
            }

        private:

            //T_MessageOptimizeCard m_message[TIME_STAMP_MSG_LEN];
            std::map<int, T_MessageOptimizeCard> m_message;
            int m_optimization_level = OPTIMIZATION_LEVEL_DEFAULT;
           
    };

    #undef TIME_STAMP_MSG_LEN

}
}
#endif