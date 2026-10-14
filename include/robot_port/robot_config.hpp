// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "robot_port/wire_codec.hpp"

namespace robot_port {
constexpr uint16_t COMMAND_PORT      = 26574;
constexpr uint16_t OPERATION_PORT    = 39244;
constexpr const char* BOARD_ADDRESS  = "192.168.3.2";
constexpr const char* JETSON_ADDRESS = "192.168.3.1";
}  // namespace robot_port
