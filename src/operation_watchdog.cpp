// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include "robot_port/operation_watchdog.hpp"

#include <stdexcept>

namespace robot_port {
OperationWatchdog::OperationWatchdog(std::chrono::nanoseconds timeout) : timeout_(timeout)
{
    if (timeout_.count() <= 0) {
        throw std::invalid_argument("Operation timeout must be positive");
    }
}

bool OperationWatchdog::neutral(const wire::Operation& operation)
{
    if ((operation.navigation_status != 0 && operation.navigation_status != 5) ||
        operation.velocity_x_m_s != 0 || operation.velocity_y_m_s != 0 ||
        operation.velocity_yaw_rad_s != 0 || operation.belt_launcher_speed_m_s != 0 ||
        operation.bucket_arm_height_m != 0 || operation.bucket_arm_width_extract ||
        operation.bucket_arm_hold) {
        return false;
    }
    for (const auto angle_deg : operation.angles_deg) {
        if (angle_deg != 0) {
            return false;
        }
    }
    return true;
}

bool OperationWatchdog::fresh(TimePoint checked, int64_t ros_now_ns)
{
    if (!have_operation_) {
        return false;
    }
    // A stopped ROS clock cannot extend a monotonic lease; rollback revokes it.
    if (checked >= deadline_ || ros_now_ns < last_ros_now_ns_ || ros_now_ns < metadata_.stamp_ns ||
        ros_now_ns - metadata_.stamp_ns >= timeout_.count()) {
        failed_                     = true;
        expired_generation_         = metadata_.generation;
        motion_generation_required_ = true;
    }
    last_ros_now_ns_ = ros_now_ns;
    return !failed_;
}

bool OperationWatchdog::accept(
    const wire::Operation& operation,
    const OperationMetadata& metadata,
    TimePoint received,
    int64_t ros_now_ns
)
{
    if (!wire::valid_operation(operation) || metadata.producer_session == 0 ||
        metadata.update_sequence == 0 || metadata.stamp_ns <= 0 || metadata.stamp_ns > ros_now_ns ||
        ros_now_ns - metadata.stamp_ns >= timeout_.count()) {
        return false;
    }
    const bool same_session =
        have_operation_ && metadata.producer_session == metadata_.producer_session;
    const bool was_fresh = fresh(received, ros_now_ns);
    if (same_session) {
        if (metadata.update_sequence <= metadata_.update_sequence ||
            metadata.generation < metadata_.generation || metadata.stamp_ns <= metadata_.stamp_ns) {
            return false;
        }
    } else {
        // No takeover during active output, and no resurrection of a retired producer.
        if (!neutral(operation) || retired_sessions_.count(metadata.producer_session) != 0 ||
            (was_fresh && !neutral(operation_))) {
            return false;
        }
        if (have_operation_) {
            retired_sessions_.insert(metadata_.producer_session);
        }
        motion_generation_required_ = false;
    }
    if (same_session && !was_fresh && !neutral(operation)) {
        return false;
    }
    if (same_session && motion_generation_required_ && !neutral(operation)) {
        if (metadata.generation <= expired_generation_) {
            return false;
        }
        motion_generation_required_ = false;
    }
    // Subtract pre-reception age: delayed snapshots never receive a full new lease.
    deadline_  = received + timeout_ - std::chrono::nanoseconds(ros_now_ns - metadata.stamp_ns);
    operation_ = operation;
    metadata_  = metadata;
    last_ros_now_ns_ = ros_now_ns;
    have_operation_  = true;
    failed_          = false;
    return true;
}

wire::Operation OperationWatchdog::current(TimePoint checked, int64_t ros_now_ns)
{
    if (fresh(checked, ros_now_ns)) {
        return operation_;
    }
    wire::Operation fallback{};
    fallback.target_id         = operation_.target_id;
    fallback.navigation_status = 5;
    return fallback;
}
}  // namespace robot_port
