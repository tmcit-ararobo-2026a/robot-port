// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace robot_port::wire {
constexpr std::size_t COMMAND_SIZE = 8;
constexpr std::size_t OPERATION_SIZE = 32;
constexpr std::size_t FEEDBACK_SIZE = 44;
constexpr uint8_t COMMAND_HEADER = 0x38;
constexpr uint8_t OPERATION_HEADER = 0xAB;
constexpr uint8_t FEEDBACK_HEADER = 0x55;

/** @brief Exact wire command fields; sequence has no request-ID semantics. */
struct Command {
  uint8_t sequence{};
  bool shutdown{};
  bool localization{};
  bool logging{};
  uint8_t target_id{};
  uint8_t navigation_command{};
  bool cloth_collect{};
};

/** @brief Operation snapshot; arrays use moving,bucket1,bucket2,bucket3,flag,desk order. */
struct Operation {
  uint8_t target_id{};
  uint8_t navigation_status{};
  float velocity_x_m_s{};
  float velocity_y_m_s{};
  float velocity_yaw_rad_s{};
  float belt_launcher_speed_m_s{};
  float bucket_arm_height_m{};
  bool bucket_arm_width_extract{};
  bool bucket_arm_hold{};
  std::array<int8_t, 6> angles_deg{};
};

/** @brief Board measurements, distinct from command acceptance. */
struct Feedback {
  uint8_t sequence{};
  bool emergency_stop_enabled{};
  bool over_current{};
  float drive_battery_voltage_v{};
  float drive_current_a{};
  float belt_launcher_target_velocity_m_s{};
  float last_belt_launcher_release_velocity_m_s{};
  std::array<float, 3> wheel_angular_velocity_rad_s{};
  float belt_launcher_velocity_m_s{};
  float loading_belt_angle_rad{};
  float bucket_arm_height_m{};
};

/** @brief Decode only a complete validated command; output unchanged on failure. */
bool decode_command(std::span<const uint8_t> bytes, Command & output);
/** @brief Decode complete finite feedback; output unchanged on failure. */
bool decode_feedback(std::span<const uint8_t> bytes, Feedback & output);
/** @brief Validate enums, finite floats and approved integer angle range. */
bool valid_operation(const Operation & operation);
/** @brief Encode explicit little-endian float32 layout; output unchanged on failure. */
bool encode_operation(const Operation & operation, std::array<uint8_t, OPERATION_SIZE> & output);
}  // namespace robot_port::wire
