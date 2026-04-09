#pragma once
#define USE_DUAL_BAND
#include <stdint.h>

namespace ethernet_config
{
    namespace main_board
    {
        uint8_t ip[4] = {192, 168, 1, 100};  // メインボードのIPアドレス
        uint16_t port_operation = 5000;      // 操作用ポート番号
        uint16_t port_feedback = 5001;       // フィードバック用ポート番号
        uint16_t port_controller_ble = 5003; // コントローラー用ポート番号 (BLE)
        uint16_t port_pid_gain = 5004;       // PIDゲイン用ポート番号
    }

    namespace pc
    {
        uint8_t ip[4] = {192, 168, 1, 12};        // PCのIPアドレス
        uint8_t ip_wifi_1[4] = {192, 168, 2, 15}; // PCのIPアドレス (WiFiモジュール)
        uint8_t ip_wifi_2[4] = {192, 168, 2, 16}; // PCのIPアドレス (WiFi-USB)
        uint16_t port_operation = 5000;           // PCの操作用ポート番号
        uint16_t port_feedback = 5001;            // PCのフィードバック用ポート番号
        uint16_t port_controller_wifi = 5002;     // コントローラー用ポート番号 (WiFi)
        uint16_t port_controller_ble = 5003;      // コントローラー用ポート番号 (BLE)
        uint16_t port_pid_gain = 5004;            // PIDゲイン用ポート番号
    }

    namespace controller
    {
        uint8_t ip[4] = {192, 168, 2, 111}; // コントローラーのIPアドレス
        uint16_t port_controller = 5002;    // コントローラー用ポート番号
    }
}

/*
#pragma once
#include "wizchip_conf.hpp"

namespace ethernet_config
{
    namespace main_board
    {
        wiz_NetInfo netInfo = {
            .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MACアドレス
            .ip = {192, 168, 1, 100},                    // IPアドレス
            .sn = {255, 255, 255, 0},                    // サブネットマスク
            .gw = {192, 168, 1, 1},                      // ゲートウェイIPアドレス
            .dns = {192, 168, 1, 1},                     // DNSサーバIPアドレス
            .dhcp = NETINFO_STATIC                       // DHCPモード（静的）
        };

        uint16_t port_operation = 5000;  // 操作用ポート番号
        uint16_t port_feedback = 5001;   // フィードバック用ポート番号
        uint16_t port_controller = 5002; // コントローラー用ポート
    }

    namespace pc
    {
        uint8_t ip[4] = {192, 168, 1, 12};      // PCのIPアドレス
        uint8_t ip_wifi[4] = {192, 168, 2, 15}; // PCのIPアドレス
        uint16_t port_operation = 5000;         // PCの操作用ポート番号
        uint16_t port_feedback = 5001;          // PCのフィードバック用ポート番号
        uint16_t port_controller = 5002;        // コントローラー用ポート
    }

    namespace controller
    {
        wiz_NetInfo netInfo = {
            .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x55}, // MACアドレス
            .ip = {192, 168, 2, 111},                    // IPアドレス
            .sn = {255, 255, 255, 0},                    // サブネットマスク
            .gw = {192, 168, 2, 1},                      // ゲートウェイIPアドレス
            .dns = {192, 168, 2, 1},                     // DNSサーバIPアドレス
            .dhcp = NETINFO_STATIC                       // DHCPモード（静的）
        };

        uint16_t port_controller = 5002; // コントローラー用ポート
    }
}*/