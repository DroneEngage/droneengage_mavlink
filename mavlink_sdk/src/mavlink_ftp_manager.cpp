#include <iostream>
#include <cstring>
#include <sstream>

#include "./helpers/colors.h"
#include "mavlink_command.h"
#include "mavlink_sdk.h"
#include "vehicle.h"
#include "mavlink_ftp_manager.h"

namespace mavlinksdk
{

CMavlinkFTPManager::CMavlinkFTPManager() 
    : m_seq_number(FTP_SEQ_NUMBER_START), m_session_count(1), m_last_error(FTP_ERROR::None)
{
}

CMavlinkFTPManager::~CMavlinkFTPManager()
{
    // Clean up any pending operations
    m_pending_operations.clear();
}

uint16_t CMavlinkFTPManager::getNextSeqNumber()
{
    return m_seq_number++;
}

uint8_t CMavlinkFTPManager::getNextSession()
{
    return m_session_count++;
}

bool CMavlinkFTPManager::sendFTPCommand(const T_PENDING_FTP& ftp_op)
{
    mavlink_file_transfer_protocol_t mav_ftp;
    mavlinksdk::CVehicle &vehicle = mavlinksdk::CVehicle::getInstance();
    
    mav_ftp.target_system = vehicle.getSysId();
    mav_ftp.target_component = vehicle.getCompId();
    mav_ftp.target_network = 0;

    // Build FTP payload
    uint8_t payload[FTP_PAYLOAD_SIZE] = {0};
    
    // FTP Header: seq_number (2 bytes), session (1 byte), opcode (1 byte), size (1 byte), offset (4 bytes), data (239 bytes)
    payload[0] = ftp_op.seq_number & 0xFF;
    payload[1] = (ftp_op.seq_number >> 8) & 0xFF;
    payload[2] = ftp_op.session;
    payload[3] = static_cast<uint8_t>(ftp_op.opcode);
    payload[4] = ftp_op.size;
    
    // Offset (4 bytes, little endian)
    payload[5] = ftp_op.offset & 0xFF;
    payload[6] = (ftp_op.offset >> 8) & 0xFF;
    payload[7] = (ftp_op.offset >> 16) & 0xFF;
    payload[8] = (ftp_op.offset >> 24) & 0xFF;
    
    // Copy data payload
    memcpy(&payload[FTP_HEADER_SIZE], ftp_op.data, FTP_DATA_SIZE);

    // Copy payload into MAVLink struct
    memcpy(mav_ftp.payload, payload, FTP_PAYLOAD_SIZE);

    // Encode and send
    mavlink_message_t msg;
    mavlink_msg_file_transfer_protocol_encode(vehicle.getSysId(), vehicle.getCompId(), &msg, &mav_ftp);
    
    // Store pending operation
    m_pending_operations[ftp_op.seq_number] = ftp_op;
    
    mavlinksdk::CMavlinkSDK::getInstance().sendMavlinkMessage(msg);
    return true;
}

void CMavlinkFTPManager::requestMavFTPParamList(FTPParamListCallback callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::OpenFileRO;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.param_callback = callback;
    ftp_op.file_path = "@PARAM/param.pck";
    
    // Set up file path in data payload
    std::string path = "@PARAM/param.pck";
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    if (!sendFTPCommand(ftp_op))
    {
        m_last_error = FTP_ERROR::Fail;
        if (callback)
        {
            callback({}, FTP_ERROR::Fail);
        }
    }
}

bool CMavlinkFTPManager::readFile(const std::string& path, FTPFileReadCallback callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::OpenFileRO;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.file_read_callback = callback;
    ftp_op.file_path = path;
    
    // Set up file path in data payload
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    return sendFTPCommand(ftp_op);
}

bool CMavlinkFTPManager::writeFile(const std::string& path, const std::vector<uint8_t>& data, FTPGenericCallback callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::CreateFile;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.generic_callback = callback;
    ftp_op.file_path = path;
    ftp_op.accumulated_data = data;
    
    // Set up file path in data payload
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    return sendFTPCommand(ftp_op);
}

bool CMavlinkFTPManager::listDirectory(const std::string& path, std::function<void(const std::vector<std::string>&, FTP_ERROR)> callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::ListDirectory;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.file_path = path;
    
    // For directory listing, we need to use a different callback type
    // Convert the generic callback to the expected format
    ftp_op.param_callback = [callback](const std::vector<std::string>& items, FTP_ERROR error) {
        if (callback) callback(items, error);
    };
    
    // Set up directory path in data payload
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    return sendFTPCommand(ftp_op);
}

bool CMavlinkFTPManager::removeFile(const std::string& path, FTPGenericCallback callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::RemoveFile;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.generic_callback = callback;
    ftp_op.file_path = path;
    
    // Set up file path in data payload
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    return sendFTPCommand(ftp_op);
}

bool CMavlinkFTPManager::createDirectory(const std::string& path, FTPGenericCallback callback)
{
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0; // New session
    ftp_op.opcode = FTP_OP::CreateDirectory;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    ftp_op.generic_callback = callback;
    ftp_op.file_path = path;
    
    // Set up directory path in data payload
    ftp_op.data[0] = static_cast<uint8_t>(path.length());
    memcpy(&ftp_op.data[1], path.c_str(), path.length());
    ftp_op.size = path.length() + 1; // Include length byte
    
    return sendFTPCommand(ftp_op);
}

void CMavlinkFTPManager::handleFTPMessage(const mavlink_message_t& msg)
{
    mavlink_file_transfer_protocol_t ftp_msg;
    mavlink_msg_file_transfer_protocol_decode(&msg, &ftp_msg);
    
    // Extract sequence number from payload
    uint16_t seq_number = ftp_msg.payload[0] | (ftp_msg.payload[1] << 8);
    
    auto it = m_pending_operations.find(seq_number);
    if (it == m_pending_operations.end())
    {
        // Unknown sequence number, ignore
        return;
    }
    
    handleFTPResponse(ftp_msg);
}

void CMavlinkFTPManager::handleFTPResponse(const mavlink_file_transfer_protocol_t& ftp_msg)
{
    // Extract FTP header
    uint16_t seq_number = ftp_msg.payload[0] | (ftp_msg.payload[1] << 8);
    uint8_t session = ftp_msg.payload[2];
    uint8_t opcode = ftp_msg.payload[3];
    uint8_t size = ftp_msg.payload[4];
    // uint32_t offset = ftp_msg.payload[5] | (ftp_msg.payload[6] << 8) | 
    //                   (ftp_msg.payload[7] << 16) | (ftp_msg.payload[8] << 24);
    
    auto it = m_pending_operations.find(seq_number);
    if (it == m_pending_operations.end())
    {
        return;
    }
    
    T_PENDING_FTP& pending_op = it->second;
    FTP_OP response_opcode = static_cast<FTP_OP>(opcode);
    
    if (response_opcode == FTP_OP::Nack)
    {
        // Handle NACK response
        FTP_ERROR error = FTP_ERROR::Fail;
        if (size < sizeof(FTP_ERROR))
        {
            error = static_cast<FTP_ERROR>(ftp_msg.payload[FTP_HEADER_SIZE]);
        }
        
        m_last_error = error;
        
        // Call appropriate callback with error
        if (pending_op.param_callback)
        {
            pending_op.param_callback({}, error);
        }
        else if (pending_op.file_read_callback)
        {
            pending_op.file_read_callback({}, error);
        }
        else if (pending_op.generic_callback)
        {
            pending_op.generic_callback(false, error);
        }
        
        // Remove pending operation
        m_pending_operations.erase(it);
        return;
    }
    
    if (response_opcode == FTP_OP::Ack)
    {
        // Handle ACK response
        FTP_ERROR error = FTP_ERROR::None;
        if (size < sizeof(FTP_ERROR))
        {
            error = static_cast<FTP_ERROR>(ftp_msg.payload[FTP_HEADER_SIZE]);
        }
        
        // Update session info
        pending_op.session = session;
        
        // Handle different operation types
        if (pending_op.opcode == FTP_OP::OpenFileRO)
        {
            // File opened successfully, now read the file
            pending_op.opcode = FTP_OP::ReadFile;
            pending_op.offset = 0;
            pending_op.size = FTP_DATA_SIZE; // Request maximum data
            
            if (!sendFTPCommand(pending_op))
            {
                m_last_error = FTP_ERROR::Fail;
                if (pending_op.param_callback)
                {
                    pending_op.param_callback({}, FTP_ERROR::Fail);
                }
                else if (pending_op.file_read_callback)
                {
                    pending_op.file_read_callback({}, FTP_ERROR::Fail);
                }
                m_pending_operations.erase(it);
            }
            return;
        }
        else if (pending_op.opcode == FTP_OP::ReadFile)
        {
            // Read file data
            std::vector<uint8_t> data(ftp_msg.payload + FTP_HEADER_SIZE, 
                                    ftp_msg.payload + FTP_HEADER_SIZE + size);
            pending_op.accumulated_data.insert(pending_op.accumulated_data.end(), 
                                             data.begin(), data.end());
            
            if (size < FTP_DATA_SIZE)
            {
                // File read complete
                if (pending_op.param_callback && pending_op.file_path == "@PARAM/param.pck")
                {
                    // Parse parameter list
                    auto params = parseParamList(pending_op.accumulated_data);
                    pending_op.param_callback(params, FTP_ERROR::None);
                }
                else if (pending_op.file_read_callback)
                {
                    pending_op.file_read_callback(pending_op.accumulated_data, FTP_ERROR::None);
                }
                
                // Close the file session
                pending_op.opcode = FTP_OP::TerminateSession;
                pending_op.size = 0;
                sendFTPCommand(pending_op);
                m_pending_operations.erase(it);
            }
            else
            {
                // Continue reading
                pending_op.offset += FTP_DATA_SIZE;
                sendFTPCommand(pending_op);
            }
            return;
        }
        else if (pending_op.opcode == FTP_OP::ListDirectory)
        {
            // Directory listing received
            std::vector<uint8_t> data(ftp_msg.payload + FTP_HEADER_SIZE, 
                                    ftp_msg.payload + FTP_HEADER_SIZE + size);
            
            // Parse directory entries (null-separated strings)
            std::vector<std::string> entries;
            std::string current;
            for (uint8_t byte : data)
            {
                if (byte == 0)
                {
                    if (!current.empty())
                    {
                        entries.push_back(current);
                        current.clear();
                    }
                }
                else
                {
                    current += static_cast<char>(byte);
                }
            }
            if (!current.empty())
            {
                entries.push_back(current);
            }
            
            if (pending_op.param_callback)
            {
                pending_op.param_callback(entries, FTP_ERROR::None);
            }
            
            m_pending_operations.erase(it);
            return;
        }
        else if (pending_op.opcode == FTP_OP::CreateFile)
        {
            // File created, now write data
            pending_op.opcode = FTP_OP::WriteFile;
            pending_op.offset = 0;
            
            // Write first chunk
            size_t chunk_size = std::min(pending_op.accumulated_data.size(), size_t(FTP_DATA_SIZE));
            memcpy(pending_op.data, pending_op.accumulated_data.data(), chunk_size);
            pending_op.size = chunk_size;
            
            if (!sendFTPCommand(pending_op))
            {
                m_last_error = FTP_ERROR::FileWriteError;
                if (pending_op.generic_callback)
                {
                    pending_op.generic_callback(false, FTP_ERROR::FileWriteError);
                }
                m_pending_operations.erase(it);
            }
            return;
        }
        else if (pending_op.opcode == FTP_OP::WriteFile)
        {
            // Data written, continue with next chunk or finish
            size_t written_size = size;
            pending_op.offset += written_size;
            
            if (pending_op.offset >= pending_op.accumulated_data.size())
            {
                // All data written, close file
                pending_op.opcode = FTP_OP::TerminateSession;
                pending_op.size = 0;
                sendFTPCommand(pending_op);
                
                if (pending_op.generic_callback)
                {
                    pending_op.generic_callback(true, FTP_ERROR::None);
                }
                m_pending_operations.erase(it);
            }
            else
            {
                // Write next chunk
                size_t remaining = pending_op.accumulated_data.size() - pending_op.offset;
                size_t chunk_size = std::min(remaining, size_t(FTP_DATA_SIZE));
                memcpy(pending_op.data, 
                       pending_op.accumulated_data.data() + pending_op.offset, 
                       chunk_size);
                pending_op.size = chunk_size;
                
                if (!sendFTPCommand(pending_op))
                {
                    m_last_error = FTP_ERROR::FileWriteError;
                    if (pending_op.generic_callback)
                    {
                        pending_op.generic_callback(false, FTP_ERROR::FileWriteError);
                    }
                    m_pending_operations.erase(it);
                }
            }
            return;
        }
        else if (pending_op.opcode == FTP_OP::RemoveFile || 
                 pending_op.opcode == FTP_OP::CreateDirectory ||
                 pending_op.opcode == FTP_OP::TerminateSession)
        {
            // Operation completed successfully
            if (pending_op.generic_callback)
            {
                pending_op.generic_callback(true, FTP_ERROR::None);
            }
            m_pending_operations.erase(it);
            return;
        }
    }
}

std::vector<std::string> CMavlinkFTPManager::parseParamList(const std::vector<uint8_t>& data)
{
    std::vector<std::string> params;
    std::string current;
    
    // Parameters are typically stored as null-terminated strings
    for (uint8_t byte : data)
    {
        if (byte == 0)
        {
            if (!current.empty())
            {
                params.push_back(current);
                current.clear();
            }
        }
        else
        {
            current += static_cast<char>(byte);
        }
    }
    
    // Add the last parameter if there's no null terminator at the end
    if (!current.empty())
    {
        params.push_back(current);
    }
    
    return params;
}

void CMavlinkFTPManager::resetSessions()
{
    // Send reset command
    T_PENDING_FTP ftp_op;
    ftp_op.seq_number = getNextSeqNumber();
    ftp_op.session = 0;
    ftp_op.opcode = FTP_OP::ResetSessions;
    ftp_op.req_opcode = FTP_OP::None;
    ftp_op.offset = 0;
    ftp_op.size = 0;
    
    sendFTPCommand(ftp_op);
    
    // Clear all pending operations
    m_pending_operations.clear();
    m_session_count = 1;
}

} // namespace mavlinksdk