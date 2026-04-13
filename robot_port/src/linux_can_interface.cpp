#include "robot_port/linux_can_interface.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

LinuxCanInterface::LinuxCanInterface(const std::string& interface_name)
    : interface_name_(interface_name)
{
}

LinuxCanInterface::~LinuxCanInterface()
{
    close();
}

bool LinuxCanInterface::open()
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    // --- CAN FDを有効化する設定を追加 ---
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        // エラー処理（インターフェースがFD非対応など）
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) return false;

    // 非ブロックモードに設定（WaitSet/Event用）
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

void LinuxCanInterface::close()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool LinuxCanInterface::send(uint32_t id, std::span<const uint8_t> data, bool is_extended)
{
    struct canfd_frame frame;
    frame.can_id = id;
    if (is_extended) frame.can_id |= CAN_EFF_FLAG;

    frame.len = static_cast<uint8_t>(std::min<size_t>(data.size(), CAN_MAX_DLEN));
    std::memcpy(frame.data, data.data(), frame.len);

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct canfd_frame));

    if (nbytes < 0) {
        if (errno == ENOBUFS) {
            // 必要に応じて内部usleepリトライを入れるか、上位にエラーを投げる
            return false;
        }
        return false;
    }
    return nbytes == sizeof(struct canfd_frame);
}

std::optional<struct canfd_frame> LinuxCanInterface::receive()
{
    struct canfd_frame frame;
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct canfd_frame));

    if (nbytes < (ssize_t)sizeof(struct canfd_frame)) {
        return std::nullopt;  // データがない、またはエラー
    }
    return frame;
}