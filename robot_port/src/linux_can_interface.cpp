#include "gn10_can/drivers/linux_can_driver.hpp"
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>

namespace gn10_can {
namespace drivers {

LinuxCANDriver::LinuxCANDriver(const std::string& interface_name)
    : interface_name_(interface_name) {}

LinuxCANDriver::~LinuxCANDriver() {
    close();
}

bool LinuxCANDriver::open() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    // CAN FD 有効化
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return false;

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) return false;

    // 非ブロックモード (WaitSet用)
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

void LinuxCANDriver::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool LinuxCANDriver::send(const FDCANFrame& frame) {
    struct canfd_frame raw_frame;
    std::memset(&raw_frame, 0, sizeof(raw_frame));

    raw_frame.can_id = frame.id;
    if (frame.is_extended) raw_frame.can_id |= CAN_EFF_FLAG;
    
    // FDフラグ (BRS: Bit Rate Switch) の適用
    if (frame.use_fd_brs) raw_frame.flags |= CANFD_BRS;

    raw_frame.len = static_cast<uint8_t>(std::min<size_t>(frame.data.size(), CANFD_MAX_DLEN));
    std::memcpy(raw_frame.data, frame.data.data(), raw_frame.len);

    ssize_t nbytes = write(socket_fd_, &raw_frame, sizeof(struct canfd_frame));
    
    if (nbytes < 0) {
        // ENOBUFS 等のエラーハンドリングが必要ならここに記述
        return false;
    }
    return nbytes == sizeof(struct canfd_frame);
}

bool LinuxCANDriver::receive(FDCANFrame& out_frame) {
    struct canfd_frame raw_frame;
    ssize_t nbytes = read(socket_fd_, &raw_frame, sizeof(struct canfd_frame));

    // Classic CAN (16 bytes) か CAN FD (72 bytes) のいずれかを受け入れる
    if (nbytes != CAN_MTU && nbytes != CANFD_MTU) {
        return false;
    }

    // FDCANFrame への変換処理
    out_frame.id = raw_frame.can_id & CAN_EFF_MASK;
    out_frame.is_extended = (raw_frame.can_id & CAN_EFF_FLAG);
    out_frame.use_fd_brs = (raw_frame.flags & CANFD_BRS);
    
    out_frame.data.assign(raw_frame.data, raw_frame.data + raw_frame.len);

    return true;
}

}  // namespace drivers
}  // namespace gn10_can