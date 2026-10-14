// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include "robot_port/operation_watchdog.hpp"

namespace robot_port {
using namespace std::chrono_literals;
using TimePoint = OperationWatchdog::TimePoint;
constexpr int64_t SECOND_NS = 1000000000;

TEST(OperationWatchdog, InitialHandshakeExpiryAndNoAutomaticRecovery) {
  OperationWatchdog watchdog(100ms);
  wire::Operation moving{};
  moving.navigation_status = 2;
  moving.velocity_x_m_s = 1;
  EXPECT_EQ(watchdog.current(TimePoint{}, SECOND_NS).navigation_status, 5);
  EXPECT_FALSE(watchdog.accept(moving, {1, 1, 1, SECOND_NS}, TimePoint{}, SECOND_NS));
  ASSERT_TRUE(watchdog.accept({}, {1, 0, 1, SECOND_NS}, TimePoint{}, SECOND_NS));
  ASSERT_TRUE(watchdog.accept(moving, {1, 1, 2, SECOND_NS + 1}, TimePoint{}, SECOND_NS + 1));
  EXPECT_FLOAT_EQ(watchdog.current(TimePoint{99ms}, SECOND_NS + 1).velocity_x_m_s, 1);
  auto failed = watchdog.current(TimePoint{100ms}, SECOND_NS + 1);
  EXPECT_EQ(failed.navigation_status, 5);
  EXPECT_FLOAT_EQ(failed.velocity_x_m_s, 0);
  EXPECT_FLOAT_EQ(failed.belt_launcher_speed_m_s, 0);
  EXPECT_FALSE(watchdog.accept(moving, {1, 1, 3, SECOND_NS + 2}, TimePoint{101ms}, SECOND_NS + 2));
  ASSERT_TRUE(watchdog.accept({}, {1, 1, 3, SECOND_NS + 2}, TimePoint{101ms}, SECOND_NS + 2));
  EXPECT_FALSE(watchdog.accept(moving, {1, 1, 4, SECOND_NS + 3}, TimePoint{102ms}, SECOND_NS + 3));
  EXPECT_TRUE(watchdog.accept(moving, {1, 2, 4, SECOND_NS + 3}, TimePoint{102ms}, SECOND_NS + 3));
}

TEST(OperationWatchdog, RejectsStaleFutureDuplicateAndRegressedMetadata) {
  OperationWatchdog watchdog(100ms);
  EXPECT_FALSE(watchdog.accept({}, {0, 0, 1, SECOND_NS}, TimePoint{}, SECOND_NS));
  EXPECT_FALSE(watchdog.accept({}, {1, 0, 0, SECOND_NS}, TimePoint{}, SECOND_NS));
  EXPECT_FALSE(watchdog.accept({}, {1, 0, 1, 0}, TimePoint{}, SECOND_NS));
  EXPECT_FALSE(watchdog.accept({}, {1, 0, 1, SECOND_NS + 1}, TimePoint{}, SECOND_NS));
  EXPECT_FALSE(watchdog.accept({}, {1, 0, 1, SECOND_NS - 100000000}, TimePoint{}, SECOND_NS));
  ASSERT_TRUE(watchdog.accept({}, {1, 2, 1, SECOND_NS}, TimePoint{}, SECOND_NS));
  EXPECT_FALSE(watchdog.accept({}, {1, 2, 1, SECOND_NS + 1}, TimePoint{}, SECOND_NS + 1));
  EXPECT_FALSE(watchdog.accept({}, {1, 1, 2, SECOND_NS + 1}, TimePoint{}, SECOND_NS + 1));
  EXPECT_FALSE(watchdog.accept({}, {1, 2, 2, SECOND_NS}, TimePoint{}, SECOND_NS + 1));
  EXPECT_FALSE(watchdog.fresh(TimePoint{100ms}, SECOND_NS + 1));
}

TEST(OperationWatchdog, ReceptionDoesNotExtendAnOldProductionDeadline) {
  OperationWatchdog watchdog(100ms);
  ASSERT_TRUE(watchdog.accept({}, {1, 1, 1, SECOND_NS - 90000000}, TimePoint{}, SECOND_NS));
  EXPECT_TRUE(watchdog.fresh(TimePoint{9ms}, SECOND_NS));
  EXPECT_FALSE(watchdog.fresh(TimePoint{10ms}, SECOND_NS));
}

TEST(OperationWatchdog, ProducerRestartAndOldSessionRejection) {
  OperationWatchdog watchdog(100ms);
  ASSERT_TRUE(watchdog.accept({}, {1, 1, 1, SECOND_NS}, TimePoint{}, SECOND_NS));
  wire::Operation moving{};
  moving.navigation_status = 2;
  moving.velocity_x_m_s = 1;
  ASSERT_TRUE(watchdog.accept(moving, {1, 2, 2, SECOND_NS + 1}, TimePoint{}, SECOND_NS + 1));
  EXPECT_FALSE(watchdog.accept({}, {2, 0, 1, SECOND_NS + 2}, TimePoint{}, SECOND_NS + 2));
  ASSERT_TRUE(watchdog.accept({}, {2, 0, 1, SECOND_NS + 2}, TimePoint{101ms}, SECOND_NS + 2));
  EXPECT_FALSE(watchdog.accept({}, {1, 3, 3, SECOND_NS + 3}, TimePoint{102ms}, SECOND_NS + 3));
}

TEST(OperationWatchdog, ClockRollbackAndForwardJumpRevokePermission) {
  OperationWatchdog watchdog(100ms);
  ASSERT_TRUE(watchdog.accept({}, {1, 0, 1, SECOND_NS}, TimePoint{}, SECOND_NS + 10));
  EXPECT_FALSE(watchdog.fresh(TimePoint{1ms}, SECOND_NS + 9));
  // A new producer session with a neutral handshake can establish a new clock epoch.
  ASSERT_TRUE(watchdog.accept({}, {2, 0, 1, SECOND_NS - 10}, TimePoint{2ms}, SECOND_NS - 10));
  EXPECT_FALSE(watchdog.fresh(TimePoint{3ms}, SECOND_NS + 100000000));
}
}  // namespace robot_port
