// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "robot_port/wire_codec.hpp"

namespace robot_port::wire {
TEST(WireCodec, OperationGoldenPacket) {
  const Operation operation{7, 3, 1.0F, -2.0F, 0.5F, 4.0F, 0.25F,
    true, false, {-20, -1, 0, 1, 19, 20}};
  const std::array<uint8_t, 32> golden{
    0xAB, 0, 7, 3, 0, 0, 0x80, 0x3F, 0, 0, 0, 0xC0,
    0, 0, 0, 0x3F, 0, 0, 0x80, 0x40, 0, 0, 0x80, 0x3E,
    1, 0, 0xEC, 0xFF, 0, 1, 19, 20};
  std::array<uint8_t, 32> bytes{};
  ASSERT_TRUE(encode_operation(operation, bytes));
  EXPECT_EQ(bytes, golden);
}

TEST(WireCodec, CommandGoldenAndInvalidValues) {
  std::array<uint8_t, 8> bytes{0x38, 0xFF, 1, 0, 1, 7, 2, 1};
  Command command{};
  ASSERT_TRUE(decode_command(bytes, command));
  EXPECT_EQ(command.sequence, 255);
  EXPECT_EQ(command.target_id, 7);
  EXPECT_TRUE(command.shutdown);
  EXPECT_FALSE(command.localization);
  EXPECT_TRUE(command.logging);
  EXPECT_TRUE(command.cloth_collect);
  for (const auto offset : {2, 3, 4, 7}) {
    auto malformed = bytes;
    malformed[offset] = 2;
    EXPECT_FALSE(decode_command(malformed, command));
  }
  bytes[5] = 8;
  EXPECT_FALSE(decode_command(bytes, command));
  bytes[5] = 7;
  bytes[6] = 3;
  EXPECT_FALSE(decode_command(bytes, command));
  EXPECT_EQ(command.navigation_command, 2);
}

TEST(WireCodec, FeedbackGoldenAndAtomicRejection) {
  // 0.0, 1.0, ... 9.0, independently encoded IEEE754 little-endian fixture.
  std::array<uint8_t, 44> bytes{0x55, 42, 1, 0,
    0, 0, 0, 0, 0, 0, 0x80, 0x3F, 0, 0, 0, 0x40,
    0, 0, 0x40, 0x40, 0, 0, 0x80, 0x40, 0, 0, 0xA0, 0x40,
    0, 0, 0xC0, 0x40, 0, 0, 0xE0, 0x40, 0, 0, 0, 0x41,
    0, 0, 0x10, 0x41};
  Feedback feedback{};
  ASSERT_TRUE(decode_feedback(bytes, feedback));
  EXPECT_EQ(feedback.sequence, 42);
  EXPECT_TRUE(feedback.emergency_stop_enabled);
  EXPECT_FALSE(feedback.over_current);
  EXPECT_FLOAT_EQ(feedback.drive_battery_voltage_v, 0);
  EXPECT_FLOAT_EQ(feedback.drive_current_a, 1);
  EXPECT_FLOAT_EQ(feedback.belt_launcher_target_velocity_m_s, 2);
  EXPECT_FLOAT_EQ(feedback.last_belt_launcher_release_velocity_m_s, 3);
  EXPECT_EQ(feedback.wheel_angular_velocity_rad_s, (std::array<float, 3>{4, 5, 6}));
  EXPECT_FLOAT_EQ(feedback.belt_launcher_velocity_m_s, 7);
  EXPECT_FLOAT_EQ(feedback.loading_belt_angle_rad, 8);
  EXPECT_FLOAT_EQ(feedback.bucket_arm_height_m, 9);
  // Corrupt the final field with NaN; earlier output fields must not change.
  bytes[1] = 43;
  bytes[42] = 0xC0;
  bytes[43] = 0x7F;
  EXPECT_FALSE(decode_feedback(bytes, feedback));
  EXPECT_EQ(feedback.sequence, 42);
  bytes[2] = 2;
  EXPECT_FALSE(decode_feedback(bytes, feedback));
}

TEST(WireCodec, RejectsLengthsHeadersAndOldFormats) {
  Command command{};
  Feedback feedback{};
  for (const auto size : {0, 7, 9, 31, 32, 43, 44, 45, 512}) {
    std::vector<uint8_t> bytes(size, 0);
    if (!bytes.empty()) {
      bytes[0] = COMMAND_HEADER;
    }
    EXPECT_FALSE(decode_command(bytes, command));
  }
  for (const auto size : {0, 8, 32, 43, 45, 512}) {
    std::vector<uint8_t> bytes(size, 0);
    if (!bytes.empty()) {
      bytes[0] = FEEDBACK_HEADER;
    }
    EXPECT_FALSE(decode_feedback(bytes, feedback));
  }
  std::array<uint8_t, 44> old_operation{};
  old_operation[0] = OPERATION_HEADER;
  EXPECT_FALSE(decode_feedback(old_operation, feedback));
  std::array<uint8_t, 8> wrong_header{};
  EXPECT_FALSE(decode_command(wrong_header, command));
}

TEST(WireCodec, InvalidOperationDoesNotChangeOutput) {
  Operation operation{};
  std::array<uint8_t, 32> bytes{};
  bytes.fill(0xAA);
  const auto original = bytes;
  operation.target_id = 8;
  EXPECT_FALSE(encode_operation(operation, bytes));
  operation.target_id = 0;
  operation.navigation_status = 6;
  EXPECT_FALSE(encode_operation(operation, bytes));
  operation.navigation_status = 0;
  operation.velocity_x_m_s = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(encode_operation(operation, bytes));
  operation.velocity_x_m_s = 0;
  operation.bucket_arm_height_m = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(encode_operation(operation, bytes));
  operation.bucket_arm_height_m = 0;
  operation.angles_deg[5] = 21;
  EXPECT_FALSE(encode_operation(operation, bytes));
  operation.angles_deg[5] = -21;
  EXPECT_FALSE(encode_operation(operation, bytes));
  EXPECT_EQ(bytes, original);
}
}  // namespace robot_port::wire
