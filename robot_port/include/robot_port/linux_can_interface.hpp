#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <optional>
#include <span>
#include <string>
#include <vector>

class LinuxCanInterface
{
public:
    explicit LinuxCanInterface(const std::string& interface_name = "can0");
    ~LinuxCanInterface();

    // コピー禁止（ソケット二重クローズ防止）
    LinuxCanInterface(const LinuxCanInterface&)            = delete;
    LinuxCanInterface& operator=(const LinuxCanInterface&) = delete;

    bool open();
    void close();

    // 送信: 成功/失敗を返す。ENOBUFS時は呼び出し側に委ねるか内部リトライ
    bool send(uint32_t id, std::span<const uint8_t> data, bool is_extended = false);

    // 受信: データがある場合のみcanfd_frameを返す
    std::optional<struct canfd_frame> receive();

    // WaitSet登録用にfdを公開
    int get_socket_fd() const
    {
        return socket_fd_;
    }

private:
    std::string interface_name_;
    int socket_fd_{-1};
};