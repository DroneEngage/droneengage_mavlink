#include "tcp_client_port.h"
#include <iostream>
#include <thread>
#include <chrono>

namespace mavlinksdk {
namespace comm {

TCPClientPort::TCPClientPort(const char* remote_ip_, int remote_port_)
{
    initialize_defaults();
    remote_ip_str = std::string(remote_ip_);
    remote_ip = remote_ip_str.c_str();
    remote_port = remote_port_;
    is_open = false;
}

TCPClientPort::TCPClientPort(const char* remote_ip_, int remote_port_, CCallBack_Communicator* callback_communicator) 
    :TCPClientPort(remote_ip_,remote_port_) {
    m_callback_communicator = callback_communicator;
    
}

TCPClientPort::TCPClientPort() {
    initialize_defaults();
}

TCPClientPort::~TCPClientPort() {
    // Destroy mutex
    pthread_mutex_destroy(&lock);
}

void TCPClientPort::initialize_defaults() {
    debug = false;
    sock_fd = -1;
    is_open = false;
    m_callback_communicator = nullptr;
    last_attempt_time = std::chrono::steady_clock::now() - std::chrono::seconds(6); // Initialize to allow immediate retry
    int result = pthread_mutex_init(&lock, NULL);
    if (result != 0) {
        printf("\n mutex init failed\n");
        throw 1;
    }
}

void TCPClientPort::start() {
    // Start a separate thread for connection retries
    std::thread([this]() {
        const int max_retries = 5;
        int retry_count = 0;
        bool connected = false;

        while (retry_count < max_retries && !connected) {
            last_attempt_time = std::chrono::steady_clock::now(); // Update attempt time
            sock_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (sock_fd < 0) {
                std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Error creating socket: " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
                retry_count++;
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            struct sockaddr_in serv_addr;
            memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(remote_port);

            if (inet_pton(AF_INET, remote_ip, &serv_addr.sin_addr) <= 0) {
                std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Invalid address/Address not supported: " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
                close(sock_fd);
                sock_fd = -1;
                retry_count++;
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            // Set a connection timeout
            struct timeval tv;
            tv.tv_sec = 5; // 5-second timeout for connect
            tv.tv_usec = 0;
            setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));

            if (connect(sock_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Connection attempt " << (retry_count + 1) << " failed: " << strerror(errno) << _NORMAL_CONSOLE_TEXT_ << std::endl;
                close(sock_fd);
                sock_fd = -1;
                retry_count++;
                if (retry_count < max_retries) {
                    std::this_thread::sleep_for(std::chrono::seconds(5));
                }
                continue;
            }

            // Connection successful
            connected = true;
            fcntl(sock_fd, F_SETFL, O_NONBLOCK); // Optional: non-blocking mode
            lastStatus.packet_rx_drop_count = 0;
            is_open = true;
            std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "TCP Client connected to " << remote_ip << ":" << remote_port << _NORMAL_CONSOLE_TEXT_ << std::endl;

            // Notify connection success
            if (m_callback_communicator) {
                m_callback_communicator->OnConnected(true);
            }
        }

        if (!connected) {
            std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "Failed to connect to " << remote_ip << ":" << remote_port << " after " << max_retries << " attempts" << _NORMAL_CONSOLE_TEXT_ << std::endl;
            // Notify connection failure
            if (m_callback_communicator) {
                m_callback_communicator->OnConnected(false);
            }
            // Exit the application
            std::exit(EXIT_FAILURE);
        }
    }).detach(); // Detach the thread to run independently
}

void TCPClientPort::stop() {
    std::cout << _INFO_CONSOLE_TEXT << "Closing TCP Client Port" << _NORMAL_CONSOLE_TEXT_ << std::endl;

    if (sock_fd != -1) {
        int result = close(sock_fd);
        if (result != 0) {
            std::cout << _ERROR_CONSOLE_BOLD_TEXT_ << "Error closing socket (" << result << ")" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        }
        sock_fd = -1;
    }

    is_open = false;
    // Notify connection status
    if (m_callback_communicator) {
        m_callback_communicator->OnConnected(false);
    }
}

int TCPClientPort::read_message(mavlink_message_t& message) {
    uint8_t cp;
    mavlink_status_t status;
    bool msgReceived = false;

    if (!is_open) {
        return false; // Don't attempt to read if not connected
    }

    int result = _read_port(cp);

    if (result > 0) {
        msgReceived = mavlink_parse_char(MAVLINK_CHANNEL_TCP, cp, &message, &status);

        if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug) {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            unsigned char v = cp;
            fprintf(stderr, "%02x ", v);
        }
        lastStatus = status;
    } else if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "ERROR: Could not read from TCP, res = %d, errno = %d : %s\n", result, errno, strerror(errno));
        stop(); // Close port on error
    }

    if (msgReceived && debug) {
        printf("Received message from TCP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
    }

    return msgReceived;
}

int TCPClientPort::_read_port(uint8_t& cp) {
    pthread_mutex_lock(&lock);

    // Set timeout for recv
    struct timeval tv;
    tv.tv_sec = 1; // 1 second timeout
    tv.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    int result = recv(sock_fd, &cp, 1, 0);

    if (result == 0) {
        // Connection closed
        std::cout << _ERROR_CONSOLE_TEXT_ << "TCP connection closed by remote" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        stop();
    } else if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Timeout, no data
        //std::this_thread::sleep_for(500);
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
        result = 0;
    }

    pthread_mutex_unlock(&lock);
    return result;
}

int TCPClientPort::write_message(const mavlink_message_t& message) {
    // Check if port is not open and last attempt was more than 5 seconds ago
    if (!is_open) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_attempt_time).count();
        if (elapsed >= 5) {
            std::cout << _INFO_CONSOLE_TEXT << "TCP port not open, attempting to reconnect..." << _NORMAL_CONSOLE_TEXT_ << std::endl;
            start(); // Retry connection in a new thread
        }
        std::cerr << _ERROR_CONSOLE_BOLD_TEXT_ << "ERROR: Cannot write, TCP port is not open or reconnecting" << _NORMAL_CONSOLE_TEXT_ << std::endl;
        return -1;
    }

    char buf[MAVLINK_MAX_PACKET_LEN];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    int bytesWritten = _write_port(buf, len);

    if (bytesWritten < 0) {
        fprintf(stderr, "ERROR: Could not write to TCP, res = %d, errno = %d : %s\n", bytesWritten, errno, strerror(errno));
        stop(); // Close port on write error
    }

    return bytesWritten;
}

int TCPClientPort::_write_port(char* buf, unsigned len) {
    pthread_mutex_lock(&lock);

    int bytesWritten = send(sock_fd, buf, len, 0);

    pthread_mutex_unlock(&lock);
    return bytesWritten;
}

} // namespace comm
} // namespace mavlinksdk