#ifndef TCP_CLIENT_PORT_H_
#define TCP_CLIENT_PORT_H_

#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <pthread.h> // POSIX Threads
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <ctime>
#include <sys/time.h>
#include <arpa/inet.h>
#include <stdexcept>
#include <chrono> // For timestamp tracking

#include <all/mavlink.h>
#include "mavlink_communicator.h" // Include for CCallBack_Communicator
#include "./helpers/colors.h"
#include "generic_port.h"

namespace mavlinksdk {
namespace comm {

class TCPClientPort : public GenericPort {
public:
    TCPClientPort(const char* remote_ip, int remote_port);
    TCPClientPort(const char* remote_ip, int remote_port, CCallBack_Communicator* callback_communicator);
    TCPClientPort();
    virtual ~TCPClientPort();

    int read_message(mavlink_message_t& message) override;
    int write_message(const mavlink_message_t& message) override;

    bool is_running() override {
        return is_open;
    }
    void start() override;
    void stop() override;

private:
    mavlink_status_t lastStatus;
    pthread_mutex_t lock;

    void initialize_defaults();

    bool debug;
    std::string remote_ip_str;
    const char* remote_ip;
    int remote_port;
    int sock_fd;
    bool is_open;
    CCallBack_Communicator* m_callback_communicator;
    std::chrono::steady_clock::time_point last_attempt_time; // Track last connection attempt

    int _read_port(uint8_t& cp);
    int _write_port(char* buf, unsigned len);
};

} // namespace comm
} // namespace mavlinksdk

#endif // TCP_CLIENT_PORT_H_