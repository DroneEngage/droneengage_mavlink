#ifndef UAVOS_MODULE_H
#define UAVOS_MODULE_H

#include <iostream>

#include "./helpers/json.hpp"
using Json = nlohmann::json;


typedef void (*SEND_JMSG_CALLBACK)(const std::string& targetPartyID, const Json&, const int&, const bool& );
typedef void (*SEND_BMSG_CALLBACK)(const std::string& targetPartyID, const char *, const int bmsg_length, const int& , const bool&);
typedef void (*SEND_MREMSG_CALLBACK)(const int& );

namespace uavos
{

    class CMODULE 
    {
        public:

            virtual void init () {};
            virtual void uninit () {};
            


            /**
             * @brief Set the PartyID & GroupID
             * 
             * @param party_id 
             * @param group_id 
             */
            virtual void setPartyID (const std::string& party_id, const std::string& group_id){};
            

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendJMSG of type @link SEND_JMSG_CALLBACK @endlink 
             */
            virtual void registerSendJMSG (SEND_JMSG_CALLBACK sendJMSG){};

            /**
             * @details register callback function to send message using it.
             * 
             * @param sendBMSG of type @link SEND_BMSG_CALLBACK @endlink 
             */
            virtual void registerSendBMSG (SEND_BMSG_CALLBACK sendBMSG){};


            /**
             * @details register call back to send InterModule remote execute message.
             * 
             * @param sendMREMSG of type @link SEND_MREMSG_CALLBACK @endlink 
             */
            virtual void registerSendMREMSG (SEND_MREMSG_CALLBACK sendMREMSG){};
            
            // called from main
            virtual void OnConnectionStatusChangedWithAndruavServer (const int status){};

            virtual  inline const Json getModuleFeatures() const
            {
                return m_module_features;
            }
        
        protected:

            /**
            * @brief module features i.e. can transmit, can recieve, needs stream ...etc.
            * in this case we use T & R only.
            */
            Json m_module_features = Json::array();
    };

};
#endif
