#ifndef MAVLINK_TLOG_HPP_
#define MAVLINK_TLOG_HPP_

#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include "../global.hpp"
#include "../helpers/colors.h"

// Check for endianness helpers based on OS
#if defined(__linux__) || defined(__APPLE__)
    #include <endian.h>
#elif defined(_WIN32)
    #include <winsock2.h>
    #define htobe64(x) htonll(x)
#endif

#include <all/mavlink.h>

namespace mavlinksdk
{
    namespace tlog
    {
        class CCallBack_MavlinkTlog
        {
            public:
                virtual void OnTlogOpened (const std::string& filename) {};
                virtual void OnTlogClosed (const std::string& filename) {};
                virtual void OnTlogError (const std::string& error_msg) {};
        };

        class CMavlinkTlog
        {
            public:
                CMavlinkTlog();
                ~CMavlinkTlog();

            public:
                bool open(const std::string& filename);
                void close();
                bool writeMessage(const mavlink_message_t& msg);
                bool isOpen() const;
                const std::string& getFilename() const;
                uint64_t getMessagesWritten() const;
                
            public:
                void set_callback_tlog(mavlinksdk::tlog::CCallBack_MavlinkTlog* callback_tlog);

            protected:
                std::ofstream m_file;
                std::string m_filename;
                uint64_t m_messages_written;
                bool m_is_open;
                mavlinksdk::tlog::CCallBack_MavlinkTlog* m_callback_tlog;

            protected:
                uint64_t getCurrentTimestampMicroseconds() const;
                uint64_t convertToBigEndian(uint64_t value) const;
        };
    }
}

#endif // MAVLINK_TLOG_HPP_