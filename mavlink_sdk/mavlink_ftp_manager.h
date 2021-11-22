#ifndef MAVLINK_FTP_MANAGER_H_
#define MAVLINK_FTP_MANAGER_H_

#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>

namespace mavlinksdk
{

 enum class FTP_OP : uint8_t {
        None = 0,
        TerminateSession = 1,
        ResetSessions = 2,
        ListDirectory = 3,
        OpenFileRO = 4,
        ReadFile = 5,
        CreateFile = 6,
        WriteFile = 7,
        RemoveFile = 8,
        CreateDirectory = 9,
        RemoveDirectory = 10,
        OpenFileWO = 11,
        TruncateFile = 12,
        Rename = 13,
        CalcFileCRC32 = 14,
        BurstReadFile = 15,
        Ack = 128,
        Nack = 129,
    };

typedef struct T_PENDING_FTP {
        uint32_t offset;
        mavlink_channel_t chan;        
        uint16_t seq_number;
        mavlinksdk::FTP_OP opcode;
        mavlinksdk::FTP_OP req_opcode;
        bool  burst_complete;
        uint8_t size;
        uint8_t session;
        uint8_t sysid;
        uint8_t compid;
        uint8_t data[239];
    } T_PENDING_FTP;

class CMavlinkFTPManager
{
    public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CMavlinkFTPManager& getInstance()
            {
                static CMavlinkFTPManager instance;
                return instance;
            }

            CMavlinkFTPManager(CMavlinkFTPManager const&)               = delete;
            void operator=(CMavlinkFTPManager const&)                   = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CMavlinkFTPManager() {};

        public:
            
            ~CMavlinkFTPManager ()
            {

            }

        public:

            void requestMavFTPParamList();

};

};

#endif
