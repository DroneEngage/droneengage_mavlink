#ifndef UAVOS_MODULE_H
#define UAVOS_MODULE_H


#include <ctime>
#include <iostream>

#include "../helpers/json.hpp"
#include "udpClient.hpp"
#include "messages.hpp"
using Json_de = nlohmann::json;

typedef enum {
    HARDWARE_TYPE_UNDEFINED     = 0,
    HARDWARE_TYPE_CPU           = 1
} ENUM_HARDWARE_TYPE;


#define MODULE_FEATURE_RECEIVING_TELEMETRY      "R"
#define MODULE_FEATURE_SENDING_TELEMETRY        "T"
#define MODULE_FEATURE_CAPTURE_IMAGE            "C"
#define MODULE_FEATURE_CAPTURE_VIDEO            "V"


#define MODULE_CLASS_FCB                        "fcb"
#define MODULE_CLASS_VIDEO                      "camera"
#define MODULE_CLASS_GENERIC                    "gen"


namespace uavos
{

namespace comm
{
    class CModule : public CCallBack_UDPClient
    {
        public:

            static CModule& getInstance()
            {
                static CModule instance;

                return instance;
            }

            CModule(CModule const&)               = delete;
            void operator=(CModule const&)        = delete;

        private:

            CModule(): cUDPClient(this)
            {
                m_instance_time_stamp = std::time(nullptr);
                m_hardware_serial = "";
                m_hardware_serial_type = HARDWARE_TYPE_UNDEFINED;

            }
        
        public:

            void defineModule (
                 std::string module_class,
                 std::string module_id,
                 std::string module_key,
                 std::string module_version,
                 Json_de message_filter
            );
            
            bool init (const std::string targetIP, int broadcatsPort, const std::string host, int listenningPort, int chunkSize) ;
            bool uninit ();
            



        public:

            void sendBMSG (const std::string& targetPartyID, const char * bmsg, const int bmsg_length, const int& andruav_message_id, const bool& internal_message, const Json_de& message_cmd);
            void sendJMSG (const std::string& targetPartyID, const Json_de& jmsg, const int& andruav_message_id, const bool& internal_message);
            void sendSYSMSG (const Json_de& jmsg, const int& andruav_message_id);
            void sendMREMSG(const int& command_type);


        public:

            void setMessageOnReceive (void (*onReceive)(const char *, int len, Json_de jMsg))
                {
                    m_OnReceive = onReceive;
                }
        
            void sendMSG (const char * msg, const int length)
                {
                    cUDPClient.sendMSG (msg, length);
                }


            void onReceive (const char *, int len) override;

        public:

            /**
             * @brief creates JSON message that identifies Module
             * @details generates JSON message that identifies module
             * 'a': module_id
             * 'b': module_class. fixed "fcb"
             * 'c': module_messages. can be updated from config file.
             * 'd': module_features. fixed per module. [T,R]
             * 'e': module_key. uniqueley identifies this instance and can be set in config file.
             * 's': hardware_serial. 
             * 't': hardware_type. 
             * 'z': resend request flag
             * @param reSend if true then server should reply with server json_msg
             */
            void createJSONID (bool reSend) ;


            /**
             * @brief Set the m_party_id & m_group_id
             * 
             * @param party_id 
             * @param group_id 
             */
            void setPartyID (const std::string& party_id, const std::string& group_id){};
            

            
            // called from main
            void OnConnectionStatusChangedWithAndruavServer (const int status){};

            /**
             * @brief Get the Module Features object
             *  module features i.e. can transmit, can recieve, needs stream ...etc.
             * in this case we use T & R only. 
             * @return const Json 
             */
             inline const Json_de getModuleFeatures() const
            {
                return m_module_features;
            }

             inline void addModuleFeatures(const std::string feature)
            {
                m_module_features.push_back(feature);
            }

            
            void setModuleClass(const std::string module_class) 
            {
                m_module_class = module_class;
            }


            void setModuleId(const std::string module_id) 
            {
                m_module_id = module_id;
            }



            const std::string getModuleKey() const
            {
                return m_module_key;
            }

            void setModuleKey(const char* module_key) 
            {
                m_module_key = module_key;
            }


            void setHardware(const std::string hardware_serial, const ENUM_HARDWARE_TYPE hardware_serial_type) 
            {
                m_hardware_serial = hardware_serial;
                m_hardware_serial_type = hardware_serial_type;
            }


            inline const std::string getGroupId() const 
            {
                return m_group_id;
            }


            inline const std::string getPartyId() const 
            {
                return m_party_id;
            }

        public:

            void appendExtraField(const std::string name, const Json_de& ms);
        
        protected:
            std::map <std::string,Json_de> m_stdinValues;
            

            

            /**
            * @brief module features i.e. can transmit, can recieve, needs stream ...etc.
            * in this case we use T & R only.
            */
            Json_de m_module_features = Json_de::array();

            /**
             * @brief Identifies the type of the module.
             * Modules of the same identical functions should have the same class
             * For example multiple camera modules should have same MODULE_CLASS_VIDEO.
             * 
             */
            std::string m_module_class;

            /**
             * @brief this id identifies this particular module. This can be 
             * FCB_Main, FCB_Secondary ....etc.
             * or CAM_MOD1, CAM_MOD2 ...etc.
             * 
             */
            std::string m_module_id;

            /**
             * @brief this is a GUID key for this module. the difference between
             * m_module_id & m_module_key is that m_module_key should be unique over
             * the whole system.
             * 
             */
            std::string m_module_key;



            std::string m_hardware_serial;


            ENUM_HARDWARE_TYPE m_hardware_serial_type;


            std::string m_module_version;

            std::time_t m_instance_time_stamp;
    
            CUDPClient cUDPClient; 

            /**
             * @brief UAVOS Current m_party_id read from communicator
             * This is important communication part to identify myself and other senders.
             */
            std::string  m_party_id;
            /**
             * @brief UAVOS Current m_group_id read from communicator
             * This is important communication part to identify myself and other senders.
             */
            std::string  m_group_id;
            
            Json_de m_message_filter;

            void (*m_OnReceive)(const char *, int len, Json_de jMsg) = nullptr;

    };
};
};
#endif
