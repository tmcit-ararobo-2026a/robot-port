#pragma once
#include <netinet/in.h>
#include <stdint.h>

#include "gn10_can/drivers/fdcan_driver_interface.hpp"

namespace gn10_can {
namespace drivers {

class LinuxUDPDriver : public IFDCANDriver
{
public:
    explicit LinuxUDPDriver(const uint8_t my_ip[4], const uint8_t dis_ip[4], const uint16_t port);
    virtual ~LinuxUDPDriver();

    LinuxUDPDriver(const LinuxUDPDriver&)            = delete;
    LinuxUDPDriver& operator=(const LinuxUDPDriver&) = delete;

    /**
     * @brief ソケットの初期化とバインド
     */
    bool open();

    /**
     * @brief ソケットのクローズ
     */
    void close();

    /**
     * @brief IFDCANDriverからの継承: フレーム送信
     */
    bool send(const FDCANFrame& frame) override;

    /**
     * @brief IFDCANDriverからの継承: フレーム受信
     */
    bool receive(FDCANFrame& out_frame) override;

    /**
     * @brief WaitSet等で使用するためのファイル記述子取得
     */
    int get_socket_fd() const
    {
        return socket_fd_;
    }

private:
    int socket_fd_{-1};
    int on_ = 1;
};

}  // namespace drivers
}  // namespace gn10_can