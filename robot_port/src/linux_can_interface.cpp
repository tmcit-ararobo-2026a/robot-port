#include "robot_port/linux_can_interface.hpp"
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

    // IDと拡張フラグの設定
    raw_frame.can_id = frame.id;
    if (frame.is_extended) {
        raw_frame.can_id |= CAN_EFF_FLAG;
    }

    // FDCANFrameのデータ長(dlc)をSocketCAN形式のlenに変換
    // 構造体で dlc が uint8_t で定義されているのでそのまま使えます
    raw_frame.len = std::min<uint8_t>(frame.dlc, static_cast<uint8_t>(CANFD_MAX_DLEN));
    
    // データのコピー
    std::memcpy(raw_frame.data, frame.data.data(), raw_frame.len);

    // FD環境なら一律でBRS（Bit Rate Switch）を有効化
    // ※もしFD/Classicを動的に切り替えたい場合は FDCANFrame に bool is_fd を追加検討
    raw_frame.flags |= CANFD_BRS;

    ssize_t nbytes = write(socket_fd_, &raw_frame, sizeof(struct canfd_frame));
    
    return nbytes == sizeof(struct canfd_frame);
}

bool LinuxCANDriver::receive(FDCANFrame& out_frame) {
    struct canfd_frame raw_frame;
    ssize_t nbytes = read(socket_fd_, &raw_frame, sizeof(struct canfd_frame));

    // Classic CAN (16 bytes) か CAN FD (72 bytes) のいずれかを受け入れる
    if (nbytes != CAN_MTU && nbytes != CANFD_MTU) {
        return false;
    }

    // IDの抽出 (フラグを除外)
    out_frame.id = raw_frame.can_id & CAN_EFF_MASK;
    out_frame.is_extended = static_cast<bool>(raw_frame.can_id & CAN_EFF_FLAG);
    
    // 受信した長さを代入
    out_frame.dlc = raw_frame.len;

    // データのコピー (std::array なので std::copy を使用)
    // 安全のため、受信した長さ(raw_frame.len)とコンテナサイズの小さい方を取る
    size_t copy_len = std::min<size_t>(static_cast<size_t>(raw_frame.len), out_frame.data.size());
    std::copy(raw_frame.data, raw_frame.data + copy_len, out_frame.data.begin());

    // 残りの領域を0で埋める（FDCANFrame::set_data の振る舞いに合わせる場合）
    if (copy_len < out_frame.data.size()) {
        std::fill(out_frame.data.begin() + copy_len, out_frame.data.end(), 0);
    }

    return true;
}

}  // namespace drivers
}  // namespace gn10_can