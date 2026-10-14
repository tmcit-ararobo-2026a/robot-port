// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <chrono>
#include <cstdint>
#include <set>

#include "robot_port/wire_codec.hpp"

namespace robot_port {
/** @brief ROS-only ordering and generation metadata, absent from the UDP layout. */
struct OperationMetadata {
    uint64_t producer_session{};
    uint64_t generation{};
    uint64_t update_sequence{};
    int64_t stamp_ns{};
};

/** @brief Single-producer snapshot admission and independent monotonic expiry. */
class OperationWatchdog
{
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    /** @brief Set the maximum age from production, not merely reception. */
    explicit OperationWatchdog(std::chrono::nanoseconds timeout);
    /** @brief Accept only a fully valid, current, ordered snapshot. */
    bool accept(
        const wire::Operation& operation,
        const OperationMetadata& metadata,
        TimePoint received,
        int64_t ros_now_ns
    );
    /** @brief Return a current snapshot or zero automatic output with Fail. */
    wire::Operation current(TimePoint checked, int64_t ros_now_ns);
    /** @brief Evaluate expiry and detect ROS clock rollback; latch failure. */
    bool fresh(TimePoint checked, int64_t ros_now_ns);
    /** @brief Test a neutral handshake without automatically operating mechanisms. */
    static bool neutral(const wire::Operation& operation);

private:
    std::chrono::nanoseconds timeout_;
    wire::Operation operation_{};
    OperationMetadata metadata_{};
    TimePoint deadline_{};
    int64_t last_ros_now_ns_{};
    bool have_operation_{};
    bool failed_{};
    bool motion_generation_required_{};
    uint64_t expired_generation_{};
    std::set<uint64_t> retired_sessions_;
};
}  // namespace robot_port
