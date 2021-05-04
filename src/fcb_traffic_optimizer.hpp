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

            void setOptimizationLevel (int level)
            {
                if (level <= OPTIMIZE_LEVEL_0) level = OPTIMIZE_LEVEL_0;
                if (level >= OPTIMIZE_LEVEL_3) level = OPTIMIZE_LEVEL_3;

                m_optimization_level = level;
            }

            /**
             * @brief Returns true if the message should be forward via telemetry to GCS.
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
                for (int i=0; i< TIME_STAMP_MSG_LEN; ++i)
                {
                    m_message[i].time_of_last_sent_message = 0;
                }

                return;
            }

        private:

            T_MessageOptimizeCard m_message[TIME_STAMP_MSG_LEN];
            int m_optimization_level = OPTIMIZATION_LEVEL_DEFAULT;
           
    };

    #undef TIME_STAMP_MSG_LEN

}
}
#endif