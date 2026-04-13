#include "robot_port/linux_can_interface.hpp"
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <cerrno>

namespace gn10_can {
namespace drivers {

LinuxCANDriver::LinuxCANDriver(const std::string& interface_name)
    : interface_name_(interface_name), socket_fd_(-1) {}

LinuxCANDriver::~LinuxCANDriver() {
    close();
}

bool LinuxCANDriver::open() {
    // 1. ソケットの作成
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        perror("gn10_can: socket");
        return false;
    }

    // 2. CAN FD 有効化 (FDCANFrameを扱うために必須)
    int enable_canfd = 1;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        perror("gn10_can: setsockopt CAN_RAW_FD_FRAMES");
        return false;
    }

    // 3. インターフェースのインデックス取得
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("gn10_can: ioctl SIOCGIFINDEX");
        return false;
    }

    // 4. アドレスのバインド
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("gn10_can: bind");
        return false;
    }

    // 5. 非ブロックモードに設定 (WaitSet/update用)
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("gn10_can: fcntl O_NONBLOCK");
    }

    return true;
}

void LinuxCANDriver::close() {
    if (socket_fd_ >= 0) {
        if (::close(socket_fd_) < 0) {
            perror("gn10_can: close");
        }
        socket_fd_ = -1;
    }
}

bool LinuxCANDriver::send(const FDCANFrame& frame) {
    struct canfd_frame raw_frame;
    std::memset(&raw_frame, 0, sizeof(raw_frame));

    raw_frame.can_id = frame.id;
    if (frame.is_extended) {
        raw_frame.can_id |= CAN_EFF_FLAG;
    }

    // FDCANの最大64バイトに対応
    raw_frame.len = std::min<uint8_t>(frame.dlc, static_cast<uint8_t>(CANFD_MAX_DLEN));
    std::memcpy(raw_frame.data, frame.data.data(), raw_frame.len);
    
    // 【重要】フラグを0にする。これで1Mbps(Nominal Bitrate)のままペイロード拡張(FD)のみ利用
    raw_frame.flags = 0; 

    ssize_t nbytes = write(socket_fd_, &raw_frame, sizeof(raw_frame));
    
    if (nbytes == -1) {
        if (errno == ENOBUFS) {
            usleep(1000); 
            nbytes = write(socket_fd_, &raw_frame, sizeof(raw_frame));
        } else {
            // ここでエラーが出た場合、ip linkの設定と矛盾しています
            // 例: EINVALなら FD が有効になっていない、Message too longならMTU不足
            // perror("gn10_can send"); 
            return false;
        }
    }

    return nbytes == sizeof(raw_frame);
}

bool LinuxCANDriver::receive(FDCANFrame& out_frame) {
    // データの読み込みが可能か確認 (selectロジック)
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(socket_fd_, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 0; // 0に設定してポーリング

    int ret = select(socket_fd_ + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0) return false;

    struct canfd_frame raw_frame;
    ssize_t nbytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));

    // SocketCANの読み込みサイズ確認 (Classic CANまたはFD)
    if (nbytes < (ssize_t)CAN_MTU) {
        return false;
    }

    // FDCANFrameへのマッピング
    out_frame.id = raw_frame.can_id & CAN_EFF_MASK;
    out_frame.is_extended = static_cast<bool>(raw_frame.can_id & CAN_EFF_FLAG);
    out_frame.dlc = raw_frame.len;

    // データコピー
    size_t copy_len = std::min<size_t>(static_cast<size_t>(raw_frame.len), out_frame.data.size());
    std::copy(raw_frame.data, raw_frame.data + copy_len, out_frame.data.begin());

    // 残りの領域をクリア
    if (copy_len < out_frame.data.size()) {
        std::fill(out_frame.data.begin() + copy_len, out_frame.data.end(), 0);
    }

    return true;
}

}  // namespace drivers
}  // namespace gn10_can