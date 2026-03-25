#ifndef MAVLINK_FTP_MANAGER_H_
#define MAVLINK_FTP_MANAGER_H_

#include <all/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>
#include <functional>
#include <string>
#include <vector>
#include <unordered_map>

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

    enum class FTP_ERROR : uint8_t {
        None = 0,
        InvalidDataLength = 1,
        Fail = 2,
        FailFileExists = 3,
        FailFileProtected = 4,
        InvalidOpcode = 5,
        InvalidParameter = 6,
        InvalidSession = 7,
        InvalidOffset = 8,
        InvalidFile = 9,
        InvalidPath = 10,
        FileWriteError = 11,
        UnhandledCommand = 12,
        FailFileDoesNotExist = 13,
    };

    // FTP Response callback types
    using FTPParamListCallback = std::function<void(const std::vector<std::string>& params, FTP_ERROR error)>;
    using FTPFileReadCallback = std::function<void(const std::vector<uint8_t>& data, FTP_ERROR error)>;
    using FTPGenericCallback = std::function<void(bool success, FTP_ERROR error)>;

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
        
        // Callbacks for async operations
        FTPParamListCallback param_callback;
        FTPFileReadCallback file_read_callback;
        FTPGenericCallback generic_callback;
        std::vector<uint8_t> accumulated_data;
        std::string file_path;
        
        T_PENDING_FTP() : offset(0), chan(static_cast<mavlink_channel_t>(0)), seq_number(0), opcode(FTP_OP::None), 
                         req_opcode(FTP_OP::None), burst_complete(false), size(0), 
                         session(0), sysid(0), compid(0) {
            memset(data, 0, sizeof(data));
        }
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

            CMavlinkFTPManager ();

            // FTP Protocol constants
            static constexpr uint8_t FTP_PAYLOAD_SIZE = 251;
            static constexpr uint8_t FTP_HEADER_SIZE = 12;
            static constexpr uint8_t FTP_DATA_SIZE = 239;
            static constexpr uint16_t FTP_SEQ_NUMBER_START = 100;
            
            // Internal state management
            std::unordered_map<uint16_t, T_PENDING_FTP> m_pending_operations;
            uint16_t m_seq_number;
            uint8_t m_session_count;
            
            // Internal helper methods
            uint16_t getNextSeqNumber();
            uint8_t getNextSession();
            bool sendFTPCommand(const T_PENDING_FTP& ftp_op);
            void handleFTPResponse(const mavlink_file_transfer_protocol_t& ftp_msg);
            std::vector<std::string> parseParamList(const std::vector<uint8_t>& data);
            
        public:
            
            ~CMavlinkFTPManager ();

            // Main FTP operations
            void requestMavFTPParamList(FTPParamListCallback callback);
            bool readFile(const std::string& path, FTPFileReadCallback callback);
            bool writeFile(const std::string& path, const std::vector<uint8_t>& data, FTPGenericCallback callback);
            bool listDirectory(const std::string& path, std::function<void(const std::vector<std::string>&, FTP_ERROR)> callback);
            bool removeFile(const std::string& path, FTPGenericCallback callback);
            bool createDirectory(const std::string& path, FTPGenericCallback callback);
            
            // Message handling - called by main SDK when FTP message is received
            void handleFTPMessage(const mavlink_message_t& msg);
            
            // Utility methods
            void resetSessions();
            FTP_ERROR getLastError() const { return m_last_error; }

        private:
            FTP_ERROR m_last_error;
};

};

#endif
