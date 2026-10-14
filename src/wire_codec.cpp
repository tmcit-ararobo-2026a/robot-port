// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include "robot_port/wire_codec.hpp"

#include <bit>
#include <cmath>
#include <limits>

namespace robot_port::wire {
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
namespace {
/** @brief Read float32 without aligned access or host-endian assumptions. */
float read_float(std::span<const uint8_t> bytes, std::size_t offset) {
  uint32_t bits = 0;
  for (std::size_t byte_index = 0; byte_index < 4; ++byte_index) {
    bits |= static_cast<uint32_t>(bytes[offset + byte_index]) << (8 * byte_index);
  }
  return std::bit_cast<float>(bits);
}

/** @brief Write float32 in STM32 little-endian byte order. */
void write_float(std::array<uint8_t, OPERATION_SIZE> & bytes, std::size_t offset, float value) {
  const auto bits = std::bit_cast<uint32_t>(value);
  for (std::size_t byte_index = 0; byte_index < 4; ++byte_index) {
    bytes[offset + byte_index] = static_cast<uint8_t>(bits >> (8 * byte_index));
  }
}
}  // namespace

bool decode_command(std::span<const uint8_t> bytes, Command & output) {
  if (bytes.size() != COMMAND_SIZE || bytes[0] != COMMAND_HEADER) {
    return false;
  }
  // Validate raw bool representation before constructing any typed value.
  for (const auto offset : {2, 3, 4, 7}) {
    if (bytes[offset] > 1) {
      return false;
    }
  }
  if (bytes[5] > 7 || bytes[6] > 2) {
    return false;
  }
  output = {bytes[1], bytes[2] == 1, bytes[3] == 1, bytes[4] == 1,
    bytes[5], bytes[6], bytes[7] == 1};
  return true;
}

bool decode_feedback(std::span<const uint8_t> bytes, Feedback & output) {
  if (bytes.size() != FEEDBACK_SIZE || bytes[0] != FEEDBACK_HEADER ||
    bytes[2] > 1 || bytes[3] > 1)
  {
    return false;
  }
  std::array<float, 10> values{};
  for (std::size_t field_index = 0; field_index < values.size(); ++field_index) {
    values[field_index] = read_float(bytes, 4 + field_index * 4);
    if (!std::isfinite(values[field_index])) {
      return false;
    }
  }
  // Publish only a fully validated snapshot.
  output = {bytes[1], bytes[2] == 1, bytes[3] == 1, values[0], values[1],
    values[2], values[3], {values[4], values[5], values[6]}, values[7], values[8], values[9]};
  return true;
}

bool valid_operation(const Operation & operation) {
  if (operation.target_id > 7 || operation.navigation_status > 5) {
    return false;
  }
  for (float value : {operation.velocity_x_m_s, operation.velocity_y_m_s,
      operation.velocity_yaw_rad_s, operation.belt_launcher_speed_m_s,
      operation.bucket_arm_height_m})
  {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (int8_t angle_deg : operation.angles_deg) {
    if (angle_deg < -20 || angle_deg > 20) {
      return false;
    }
  }
  return true;
}

bool encode_operation(const Operation & operation, std::array<uint8_t, OPERATION_SIZE> & output) {
  if (!valid_operation(operation)) {
    return false;
  }
  std::array<uint8_t, OPERATION_SIZE> bytes{};
  bytes[0] = OPERATION_HEADER;
  bytes[2] = operation.target_id;
  bytes[3] = operation.navigation_status;
  write_float(bytes, 4, operation.velocity_x_m_s);
  write_float(bytes, 8, operation.velocity_y_m_s);
  write_float(bytes, 12, operation.velocity_yaw_rad_s);
  write_float(bytes, 16, operation.belt_launcher_speed_m_s);
  write_float(bytes, 20, operation.bucket_arm_height_m);
  bytes[24] = static_cast<uint8_t>(operation.bucket_arm_width_extract);
  bytes[25] = static_cast<uint8_t>(operation.bucket_arm_hold);
  for (std::size_t angle_index = 0; angle_index < operation.angles_deg.size(); ++angle_index) {
    bytes[26 + angle_index] = std::bit_cast<uint8_t>(operation.angles_deg[angle_index]);
  }
  output = bytes;
  return true;
}
}  // namespace robot_port::wire
