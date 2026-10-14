// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <system_error>
#include <thread>
#include <vector>

#include "robot_port/simple_udp.hpp"

namespace robot_port {
/** @brief Wait briefly for a local datagram without a blocking socket. */
DatagramResult receive_eventually(UDPChannel & channel, std::span<uint8_t> buffer) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    auto result = channel.receive(buffer);
    if (result.status != DatagramResult::Status::NO_DATA) {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return {};
}

TEST(UDPChannel, ExplicitDestinationSourceAndFullLength) {
  UDPChannel receiver({"127.0.0.1", 0}, {"127.0.0.1", 1});
  UDPChannel configured_receiver({"127.0.0.1", 0}, {"127.0.0.1", 1});
  UDPChannel sender({"127.0.0.1", 0}, configured_receiver.local_endpoint());
  std::vector<uint8_t> data(512, 0xAB);
  ASSERT_TRUE(sender.send_to(data, receiver.local_endpoint()));
  std::array<uint8_t, 44> buffer{};
  const auto result = receive_eventually(receiver, buffer);
  ASSERT_EQ(result.status, DatagramResult::Status::RECEIVED);
  EXPECT_EQ(result.size, 512);
  EXPECT_TRUE(result.truncated);
  EXPECT_EQ(result.source, sender.local_endpoint());
  EXPECT_EQ(configured_receiver.receive(buffer).status, DatagramResult::Status::NO_DATA);
  ASSERT_TRUE(sender.send(std::span<const uint8_t>(data.data(), 32)));
  const auto configured_result = receive_eventually(configured_receiver, buffer);
  EXPECT_EQ(configured_result.size, 32);
  EXPECT_FALSE(configured_result.truncated);
}

TEST(UDPChannel, EmptyPacketIsNotNoData) {
  UDPChannel receiver({"127.0.0.1", 0}, {"127.0.0.1", 1});
  UDPChannel sender({"127.0.0.1", 0}, receiver.local_endpoint());
  std::array<uint8_t, 8> buffer{};
  ASSERT_TRUE(sender.send({}));
  const auto result = receive_eventually(receiver, buffer);
  EXPECT_EQ(result.status, DatagramResult::Status::RECEIVED);
  EXPECT_EQ(result.size, 0);
  EXPECT_EQ(receiver.receive(buffer).status, DatagramResult::Status::NO_DATA);
}

TEST(UDPChannel, ExclusiveBindAndDestructorRelease) {
  Endpoint endpoint;
  {
    UDPChannel owner({"127.0.0.1", 0}, {"127.0.0.1", 1});
    endpoint = owner.local_endpoint();
    EXPECT_THROW((UDPChannel(endpoint, {"127.0.0.1", 1})), std::system_error);
  }
  EXPECT_NO_THROW((UDPChannel(endpoint, {"127.0.0.1", 1})));
  EXPECT_THROW((UDPChannel({"invalid", 0}, {"127.0.0.1", 1})), std::invalid_argument);
}
}  // namespace robot_port
