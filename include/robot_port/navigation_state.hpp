// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace robot_port {
/** @brief Board-visible state values; preserve the firmware spelling and numeric contract. */
enum class NavigationStatus : uint8_t {
    Stanby   = 0,
    Ready    = 1,
    Moving   = 2,
    Tracking = 3,
    Goal     = 4,
    Fail     = 5
};

/** @brief Latch only explicit localization/navigation failures, without clocks or leases. */
class NavigationState
{
public:
    /** @brief Record localization availability; an explicit false always latches Fail. */
    void observe_localization(bool valid)
    {
        have_localization_  = true;
        localization_valid_ = valid;
        if (!valid) {
            enter_fail();
            return;
        }
        refresh();
    }

    /** @brief Record a validated navigation phase; Fail denotes a non-continuable failure. */
    void observe_navigation(NavigationStatus status)
    {
        have_navigation_ = true;
        navigation_      = status;
        if (status == NavigationStatus::Fail) {
            enter_fail();
            return;
        }
        waiting_navigation_ = false;
        refresh();
    }

    /** @brief Recover only on an observed Sleep/Stanby-to-Move edge after both inputs recover. */
    bool observe_command(uint8_t command)
    {
        const bool new_move = have_command_ && previous_command_ != 2 && command == 2;
        previous_command_   = command;
        have_command_       = true;
        if (!failed_ || !healthy() || !new_move) {
            return false;
        }
        failed_             = false;
        waiting_navigation_ = true;
        status_             = NavigationStatus::Ready;
        return true;
    }

    /** @brief Return the effective board state, including any latched failure. */
    NavigationStatus status() const
    {
        return status_;
    }

    /** @brief Require known healthy inputs and a fresh phase after failure recovery. */
    bool ready() const
    {
        return healthy() && !failed_ && !waiting_navigation_;
    }

    /** @brief Chassis velocity is permitted only in Moving or Tracking. */
    bool motion_allowed() const
    {
        return ready() &&
               (status_ == NavigationStatus::Moving || status_ == NavigationStatus::Tracking);
    }

private:
    /** @brief Unknown inputs are not healthy and are not failures. */
    bool healthy() const
    {
        return have_localization_ && localization_valid_ && have_navigation_ &&
               navigation_ != NavigationStatus::Fail;
    }

    /** @brief Preserve Fail until explicit input recovery and a new board Move request. */
    void enter_fail()
    {
        failed_             = true;
        waiting_navigation_ = false;
        status_             = NavigationStatus::Fail;
    }

    /** @brief Follow normal navigation phases only after both inputs are known and healthy. */
    void refresh()
    {
        if (failed_ || waiting_navigation_) {
            return;
        }
        status_ = NavigationStatus::Stanby;
        if (healthy()) {
            status_ = navigation_;
        }
    }

    NavigationStatus status_{NavigationStatus::Stanby};
    NavigationStatus navigation_{NavigationStatus::Stanby};
    bool have_localization_{};
    bool localization_valid_{};
    bool have_navigation_{};
    bool failed_{};
    bool waiting_navigation_{};
    bool have_command_{};
    uint8_t previous_command_{};
};
}  // namespace robot_port
