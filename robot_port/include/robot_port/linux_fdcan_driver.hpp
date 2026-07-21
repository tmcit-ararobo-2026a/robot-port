/**
 * @file linux_can_driver.hpp
 * @author Gento Aiba
 * @brief SocketCANを用いたLinux用CAN FDドライバークラス
 */
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <string>

#include "gn10_can/drivers/fdcan_driver_interface.hpp"

namespace gn10_can {
namespace drivers {

class LinuxFDCANDriver : public IFDCANDriver
{
public:
    explicit LinuxFDCANDriver(const std::string& interface_name = "can0");
    virtual ~LinuxFDCANDriver();

    // コピー禁止
    LinuxFDCANDriver(const LinuxFDCANDriver&)            = delete;
    LinuxFDCANDriver& operator=(const LinuxFDCANDriver&) = delete;

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
    std::string interface_name_;
    int socket_fd_{-1};
};

}  // namespace drivers
}  // namespace gn10_can