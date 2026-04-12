/**
 * @file can_peripheral.hpp
 * @author Gento Aiba
 * @brief CAN通信をLinuxで行うクラス
 * @version 1.0
 * @date 2024-03-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CANPeripheral
{
private:
    int can_socket;
    int rx_numbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame can_rx_frame;
    struct can_frame can_tx_frame;

protected:
    /**
     * @brief 受信するデータがあるかどうかを返す関数
     *
     * @return true
     * @return false
     */
    bool isDataAvailable();

    /**
     * @brief データーを送信する関数
     *
     * @param id CANのID
     * @param send_buffer 送信するデータ配列(uint8)のポインタ
     * @param data_length 送信するデータの大きさ
     */
    void sendPacket(uint16_t id, uint8_t send_buffer[], uint8_t data_length);

    /**
     * @brief データを受信する関数
     *
     * @param receive_buffer 受信したデータを入れるバッファ(8byte)
     * @param rx_id 受信したデータのCANのID
     * @param rx_data_length 受信したデータの大きさ
     */
    void readPacket(uint8_t *receive_buffer, uint16_t *rx_id, uint8_t *rx_data_length);

public:
    /**
     * @brief CAN通信を初期化
     *
     */
    void initCAN();

    /**
     * @brief CAN通信を終了
     *
     */
    void closeSocket();
};