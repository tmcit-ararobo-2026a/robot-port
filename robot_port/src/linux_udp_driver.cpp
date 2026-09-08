#include "robot_port/linux_udp_driver.hpp"

#include <arpa/inet.h>
#include <errno.h>  // For errno and EWOULDBLOCK
#include <fcntl.h>  // For fcntl()
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <memory>

namespace gn10_can {
namespace drivers {

LinuxUDPDriver::LinuxUDPDriver() : socket_fd_(-1) {}

LinuxUDPDriver::~LinuxUDPDriver()
{
    close();
}

bool LinuxUDPDriver::open()
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        perror("gn10_can: socket");
        return false;
    }

    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &on_, sizeof(on_));

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}
bool LinuxUDPDriver::bind(uint16_t port)
{
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);  // すべてのIPからの受信を許可
    addr.sin_port        = htons(port);

    if (::bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("gn10_can: bind");
        return false;
    }
    return true;
}

ssize_t LinuxUDPDriver::receive(uint8_t* buffer, size_t max_size)
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    // 非ブロックモードなので、データがなければ -1 (EWOULDBLOCK) が返る
    ssize_t nbytes =
        recvfrom(socket_fd_, buffer, max_size, 0, (struct sockaddr*)&client_addr, &addr_len);

    return nbytes;  // 受信したバイト数（データがなければマイナス）
}

}  // namespace drivers
}  // namespace gn10_can