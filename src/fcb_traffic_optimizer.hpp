#ifndef FCB_TRAFFIC_OPTIIZER_H_

#define FCB_TRAFFIC_OPTIIZER_H_

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

    typedef struct T_MessageOptimizeCard
    {
        int timeout[OPTIMIZE_LEVELS];
        std::uint64_t time_of_last_sent_message;
    } T_MessageOptimizeCard;


    class CMavlinkTrafficOptimizer
    {

        public:

        static T_MessageOptimizeCard m_message[TIME_STAMP_MSG_LEN];


        static void reset_timestamps()
        {
            for (int i=0; i< TIME_STAMP_MSG_LEN; ++i)
            {
                m_message[i].time_of_last_sent_message = 0;
            }

            return;
        }


        static bool ShouldForwardThisMessage (const mavlink_message_t& mavlink_message)
        {
            return false;
        }
    };

    #undef TIME_STAMP_MSG_LEN

}
}
#endif