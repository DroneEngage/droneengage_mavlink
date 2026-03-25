#include "mavlink_tlog.hpp"
#include <iostream>

namespace mavlinksdk
{
namespace tlog
{

CMavlinkTlog::CMavlinkTlog()
{
    m_messages_written = 0;
    m_is_open = false;
    m_callback_tlog = nullptr;
}

CMavlinkTlog::~CMavlinkTlog()
{
    close();
}

bool CMavlinkTlog::open(const std::string& filename)
{
    if (m_is_open)
    {
        close();
    }

    m_filename = filename;
    m_file.open(filename, std::ios::binary | std::ios::out | std::ios::app);
    m_is_open = m_file.is_open();
    
    if (m_is_open)
    {
        m_messages_written = 0;
        std::cout << _INFO_CONSOLE_TEXT << "TLOG file opened: " << filename << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        if (m_callback_tlog != nullptr)
        {
            m_callback_tlog->OnTlogOpened(filename);
        }
        return true;
    }
    else
    {
        std::cout << _ERROR_CONSOLE_TEXT_ << "Failed to open TLOG file: " << filename << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        if (m_callback_tlog != nullptr)
        {
            m_callback_tlog->OnTlogError("Failed to open file: " + filename);
        }
        return false;
    }
}

void CMavlinkTlog::close()
{
    if (m_is_open && m_file.is_open())
    {
        m_file.close();
        m_is_open = false;
        
        std::cout << _INFO_CONSOLE_TEXT << "TLOG file closed: " << m_filename 
                  << " (Messages written: " << m_messages_written << ")" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        if (m_callback_tlog != nullptr)
        {
            m_callback_tlog->OnTlogClosed(m_filename);
        }
    }
}

bool CMavlinkTlog::writeMessage(const mavlink_message_t& msg)
{
    if (!m_is_open || !m_file.is_open())
    {
        if (m_callback_tlog != nullptr)
        {
            m_callback_tlog->OnTlogError("Attempted to write to closed TLOG file");
        }
        return false;
    }

    try
    {
        // 1. Get Unix Timestamp (Microseconds)
        uint64_t now_us = getCurrentTimestampMicroseconds();

        // 2. Convert to Big-Endian for Mission Planner compatibility
        uint64_t timestamp_be = convertToBigEndian(now_us);

        // 3. Serialize MAVLink message to buffer
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

        // 4. Write: [8 bytes timestamp] + [Raw MAVLink packet]
        m_file.write(reinterpret_cast<const char*>(&timestamp_be), sizeof(timestamp_be));
        m_file.write(reinterpret_cast<const char*>(buffer), len);

        // Flush occasionally to ensure data isn't lost on crash
        m_file.flush();
        
        if (m_file.good())
        {
            m_messages_written++;
            return true;
        }
        else
        {
            std::cout << _ERROR_CONSOLE_TEXT_ << "Failed to write message to TLOG file" << _NORMAL_CONSOLE_TEXT_ << std::endl;
            
            if (m_callback_tlog != nullptr)
            {
                m_callback_tlog->OnTlogError("Failed to write message to TLOG file");
            }
            return false;
        }
    }
    catch (const std::exception& e)
    {
        std::cout << _ERROR_CONSOLE_TEXT_ << "Exception in writeMessage: " << e.what() << _NORMAL_CONSOLE_TEXT_ << std::endl;
        
        if (m_callback_tlog != nullptr)
        {
            m_callback_tlog->OnTlogError(std::string("Exception in writeMessage: ") + e.what());
        }
        return false;
    }
}

bool CMavlinkTlog::isOpen() const
{
    return m_is_open;
}

const std::string& CMavlinkTlog::getFilename() const
{
    return m_filename;
}

uint64_t CMavlinkTlog::getMessagesWritten() const
{
    return m_messages_written;
}

void CMavlinkTlog::set_callback_tlog(mavlinksdk::tlog::CCallBack_MavlinkTlog* callback_tlog)
{
    m_callback_tlog = callback_tlog;
}

uint64_t CMavlinkTlog::getCurrentTimestampMicroseconds() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

uint64_t CMavlinkTlog::convertToBigEndian(uint64_t value) const
{
    return htobe64(value);
}

} // namespace tlog
} // namespace mavlinksdk