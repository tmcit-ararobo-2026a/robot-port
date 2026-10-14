// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "robot_port/msg/bridge_health.hpp"
#include "robot_port/msg/robot_command.hpp"
#include "robot_port/msg/robot_feedback.hpp"
#include "robot_port/msg/robot_operation.hpp"
#include "robot_port/operation_watchdog.hpp"
#include "robot_port/robot_config.hpp"
#include "robot_port/simple_udp.hpp"

namespace robot_port {
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using namespace std::chrono_literals;

/** @brief Validated two-channel UDP bridge with exclusively owned sockets. */
class UDPNode : public rclcpp::Node
{
public:
    /** @brief Validate parameters and bind both sockets before installing callbacks. */
    UDPNode() : Node("udp_node")
    {
        const auto local_address  = declare_parameter<std::string>("local_address", JETSON_ADDRESS);
        const auto peer_address   = declare_parameter<std::string>("peer_address", BOARD_ADDRESS);
        const auto command_port   = port_parameter("command_port", COMMAND_PORT);
        const auto operation_port = port_parameter("operation_port", OPERATION_PORT);
        if (command_port == operation_port) {
            throw std::invalid_argument("command_port and operation_port must differ");
        }
        transmission_enabled_ = declare_parameter<bool>("enable_transmission", false);
        test_mode_            = declare_parameter<bool>("test_mode", false);
        const bool test_mode  = test_mode_;
        loopback_only_ = local_address.rfind("127.", 0) == 0 && peer_address.rfind("127.", 0) == 0;
        std::random_device random;
        bridge_session_ = (static_cast<uint64_t>(random()) << 32) | random();
        if (bridge_session_ == 0) {
            bridge_session_ = 1;
        }
        const bool contract_confirmed =
            declare_parameter<bool>("physical_contract_confirmed", false);
        // Fixtures can never target a physical interface, even if transmission is off.
        if (test_mode &&
            (local_address.rfind("127.", 0) != 0 || peer_address.rfind("127.", 0) != 0)) {
            throw std::invalid_argument("test_mode requires loopback local and peer addresses");
        }
        if (transmission_enabled_ && !test_mode && !contract_confirmed) {
            throw std::invalid_argument("Physical transmission requires confirmed external contract"
            );
        }
        command_peer_  = {peer_address, command_port};
        feedback_peer_ = {peer_address, operation_port};
        command_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local_address, command_port}, command_peer_);
        operation_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local_address, operation_port}, feedback_peer_);

        command_publisher_  = create_publisher<msg::RobotCommand>("/robot/command", 10);
        feedback_publisher_ = create_publisher<msg::RobotFeedback>("/robot/feedback", 10);
        health_publisher_ = create_publisher<msg::BridgeHealth>("/robot/communication_status", 10);
        operation_subscription_ = create_subscription<msg::RobotOperation>(
            "/robot/operation",
            rclcpp::QoS(1).durability_volatile(),
            [this](msg::RobotOperation::ConstSharedPtr message) { accept_operation(*message); }
        );
        operation_timeout_ms_ = positive_parameter("operation_timeout_ms", 100);
        if (operation_timeout_ms_ > std::numeric_limits<int64_t>::max() / 1000000) {
            throw std::invalid_argument("operation_timeout_ms exceeds nanosecond range");
        }
        watchdog_ =
            std::make_unique<OperationWatchdog>(std::chrono::milliseconds(operation_timeout_ms_));
        command_timeout_ms_  = positive_parameter("command_timeout_ms", 150);
        feedback_timeout_ms_ = positive_parameter("feedback_timeout_ms", 150);
        receive_timer_       = create_wall_timer(5ms, [this]() { receive_packets(); });
        send_timer_          = create_wall_timer(20ms, [this]() { transmit(); });
        health_timer_        = create_wall_timer(20ms, [this]() { publish_health(); });
        RCLCPP_INFO(
            get_logger(),
            "New UDP bridge: command=%u operation=%u transmit=%d",
            command_port,
            operation_port,
            transmission_enabled_
        );
    }

private:
    /** @brief Read a positive timeout, rejecting incomplete/invalid setup. */
    int64_t positive_parameter(const std::string& name, int64_t initial)
    {
        const auto value = declare_parameter<int64_t>(name, initial);
        if (value <= 0) {
            throw std::invalid_argument(name + " must be positive");
        }
        return value;
    }
    /** @brief Read a valid nonzero UDP port. */
    uint16_t port_parameter(const std::string& name, uint16_t initial)
    {
        const auto value = positive_parameter(name, initial);
        if (value > 65535) {
            throw std::invalid_argument(name + " exceeds 65535");
        }
        return static_cast<uint16_t>(value);
    }
    /** @brief Convert a complete ROS snapshot to validated wire fields. */
    void accept_operation(const msg::RobotOperation& message)
    {
        wire::Operation operation{};
        operation.target_id                = message.target_id;
        operation.navigation_status        = message.navigation_status;
        operation.velocity_x_m_s           = message.velocity_x_m_s;
        operation.velocity_y_m_s           = message.velocity_y_m_s;
        operation.velocity_yaw_rad_s       = message.velocity_yaw_rad_s;
        operation.belt_launcher_speed_m_s  = message.belt_launcher_speed_m_s;
        operation.bucket_arm_height_m      = message.bucket_arm_height_m;
        operation.bucket_arm_width_extract = message.bucket_arm_width_extract;
        operation.bucket_arm_hold          = message.bucket_arm_hold;
        operation.angles_deg               = message.angles_deg;

        if (message.stamp.nanosec >= 1000000000U || message.stamp.sec < 0 ||
            !watchdog_->accept(
                operation,
                {message.producer_session,
                 message.generation,
                 message.update_sequence,
                 rclcpp::Time(message.stamp).nanoseconds()},
                Clock::now(),
                now().nanoseconds()
            )) {
            ++rejected_operations_;
            return;
        }
        accepted_session_    = message.producer_session;
        accepted_generation_ = message.generation;
        operation_received_  = Clock::now();
        have_operation_      = true;
    }
    /** @brief Drain bounded batches so packet floods cannot starve transmission. */
    void receive_packets()
    {
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, wire::COMMAND_SIZE> bytes{};
            const auto result = command_channel_->receive(bytes);
            if (result.status == DatagramResult::Status::NO_DATA) {
                break;
            }
            if (result.status == DatagramResult::Status::ERROR) {
                ++receive_errors_;
                break;
            }
            wire::Command command{};
            if (result.source != command_peer_ || result.truncated || result.size != bytes.size() ||
                !wire::decode_command(bytes, command)) {
                ++rejected_command_packets_;
                continue;
            }
            command_received_ = Clock::now();
            have_command_     = true;
            msg::RobotCommand message;
            message.received_stamp     = now();
            message.sequence           = command.sequence;
            message.shutdown           = command.shutdown;
            message.localization       = command.localization;
            message.logging            = command.logging;
            message.target_id          = command.target_id;
            message.navigation_command = command.navigation_command;
            message.cloth_collect      = command.cloth_collect;

            command_publisher_->publish(message);
        }
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, wire::FEEDBACK_SIZE> bytes{};
            const auto result = operation_channel_->receive(bytes);
            if (result.status == DatagramResult::Status::NO_DATA) {
                break;
            }
            if (result.status == DatagramResult::Status::ERROR) {
                ++receive_errors_;
                break;
            }
            wire::Feedback feedback{};
            if (result.source != feedback_peer_ || result.truncated ||
                result.size != bytes.size() || !wire::decode_feedback(bytes, feedback)) {
                ++rejected_feedback_packets_;
                continue;
            }
            feedback_received_ = Clock::now();
            have_feedback_     = true;
            msg::RobotFeedback message;
            message.received_stamp                    = now();
            message.sequence                          = feedback.sequence;
            message.emergency_stop_enabled            = feedback.emergency_stop_enabled;
            message.over_current                      = feedback.over_current;
            message.drive_battery_voltage_v           = feedback.drive_battery_voltage_v;
            message.drive_current_a                   = feedback.drive_current_a;
            message.belt_launcher_target_velocity_m_s = feedback.belt_launcher_target_velocity_m_s;
            message.last_belt_launcher_release_velocity_m_s =
                feedback.last_belt_launcher_release_velocity_m_s;
            message.wheel_angular_velocity_rad_s = feedback.wheel_angular_velocity_rad_s;
            message.belt_launcher_velocity_m_s   = feedback.belt_launcher_velocity_m_s;
            message.loading_belt_angle_rad       = feedback.loading_belt_angle_rad;
            message.bucket_arm_height_m          = feedback.bucket_arm_height_m;

            feedback_publisher_->publish(message);
        }
    }
    /** @brief Send one complete 32-byte operation from the feedback-bound socket. */
    void transmit()
    {
        if (!transmission_enabled_) {
            return;
        }
        std::array<uint8_t, wire::OPERATION_SIZE> bytes{};
        const auto operation = watchdog_->current(Clock::now(), now().nanoseconds());
        if (!wire::encode_operation(operation, bytes)) {
            ++send_errors_;
            last_send_success_ = false;
            return;
        }
        last_send_success_ = operation_channel_->send(bytes);
        if (last_send_success_) {
            last_sent_session_    = accepted_session_;
            last_sent_generation_ = accepted_generation_;
            last_sent_status_     = operation.navigation_status;
            last_successful_send_ = Clock::now();
            have_successful_send_ = true;
        } else {
            ++send_errors_;
        }
    }
    /** @brief Report a monotonic age or -1 when no accepted event exists. */
    double age_ms(bool have_event, TimePoint event) const
    {
        if (!have_event) {
            return -1.0;
        }
        return std::chrono::duration<double, std::milli>(Clock::now() - event).count();
    }
    /** @brief Report independent receive freshness and actual send results. */
    void publish_health()
    {
        msg::BridgeHealth health;
        health.stamp                       = now();
        health.last_sent_producer_session  = last_sent_session_;
        health.last_sent_generation        = last_sent_generation_;
        health.last_sent_navigation_status = last_sent_status_;
        health.bridge_session              = bridge_session_;
        health.update_sequence             = ++health_sequence_;
        health.test_mode                   = test_mode_;
        health.loopback_only               = loopback_only_;
        health.command_age_ms              = age_ms(have_command_, command_received_);
        health.feedback_age_ms             = age_ms(have_feedback_, feedback_received_);
        health.operation_age_ms            = age_ms(have_operation_, operation_received_);
        health.last_successful_send_age_ms = age_ms(have_successful_send_, last_successful_send_);
        health.command_fresh   = have_command_ && health.command_age_ms < command_timeout_ms_;
        health.feedback_fresh  = have_feedback_ && health.feedback_age_ms < feedback_timeout_ms_;
        health.operation_fresh = watchdog_->fresh(Clock::now(), now().nanoseconds());
        health.watchdog_active = !health.operation_fresh;
        health.transmission_enabled      = transmission_enabled_;
        health.last_send_success         = last_send_success_;
        health.rejected_command_packets  = rejected_command_packets_;
        health.rejected_feedback_packets = rejected_feedback_packets_;
        health.rejected_operations       = rejected_operations_;
        health.receive_errors            = receive_errors_;
        health.send_errors               = send_errors_;
        health.reason = "UDP transmission is not board acceptance or physical stop proof";
        health_publisher_->publish(health);
    }

    std::unique_ptr<UDPChannel> command_channel_;
    std::unique_ptr<UDPChannel> operation_channel_;
    Endpoint command_peer_;
    Endpoint feedback_peer_;
    std::unique_ptr<OperationWatchdog> watchdog_;
    bool transmission_enabled_{};
    bool last_send_success_{};
    bool have_command_{};
    bool have_feedback_{};
    bool have_operation_{};
    bool have_successful_send_{};
    TimePoint command_received_{};
    TimePoint feedback_received_{};
    TimePoint operation_received_{};
    TimePoint last_successful_send_{};
    int64_t operation_timeout_ms_{};
    int64_t command_timeout_ms_{};
    int64_t feedback_timeout_ms_{};
    uint64_t rejected_command_packets_{};
    uint64_t rejected_feedback_packets_{};
    uint64_t rejected_operations_{};
    uint64_t receive_errors_{};
    uint64_t send_errors_{};
    rclcpp::Publisher<msg::RobotCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<msg::RobotFeedback>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<msg::BridgeHealth>::SharedPtr health_publisher_;
    uint64_t bridge_session_{}, health_sequence_{};
    uint64_t accepted_session_{}, accepted_generation_{}, last_sent_session_{},
        last_sent_generation_{};
    uint8_t last_sent_status_{};
    bool test_mode_{}, loopback_only_{};
    rclcpp::Subscription<msg::RobotOperation>::SharedPtr operation_subscription_;
    rclcpp::TimerBase::SharedPtr receive_timer_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
};
}  // namespace robot_port

/** @brief Start the bridge; invalid socket/configuration causes a visible failure. */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<robot_port::UDPNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("udp_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
