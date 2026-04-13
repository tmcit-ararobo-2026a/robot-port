/**
 * @file linux_can_driver.hpp
 * @author Gento Aiba
 * @brief SocketCANを用いたLinux用CAN FDドライバークラス
 */
#pragma once

#include "gn10_can/drivers/fdcan_driver_interface.hpp"
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace gn10_can {
namespace drivers {

class LinuxCANDriver : public IFDCANDriver
{
public:
    explicit LinuxCANDriver(const std::string& interface_name = "can0");
    virtual ~LinuxCANDriver();

    // コピー禁止
    LinuxCANDriver(const LinuxCANDriver&) = delete;
    LinuxCANDriver& operator=(const LinuxCANDriver&) = delete;

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
    int get_socket_fd() const { return socket_fd_; }

private:
    std::string interface_name_;
    int socket_fd_{-1};
};

}  // namespace drivers
}  // namespace gn10_can