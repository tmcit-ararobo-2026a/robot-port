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

}  // namespace drivers
}  // namespace gn10_can