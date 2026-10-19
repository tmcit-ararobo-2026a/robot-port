// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <stdexcept>
#include <vector>

#include "robot_port/navigation_state.hpp"
#include "robot_port/robot_config.hpp"
#include "robot_port/simple_udp.hpp"

namespace robot_port {
using namespace std::chrono_literals;
using Bool         = std_msgs::msg::Bool;
using Float32      = std_msgs::msg::Float32;
using Float32Array = std_msgs::msg::Float32MultiArray;
using UInt8        = std_msgs::msg::UInt8;
using UInt8Array   = std_msgs::msg::UInt8MultiArray;
using Int8Array    = std_msgs::msg::Int8MultiArray;
using Twist        = geometry_msgs::msg::Twist;

/** @brief Update independent standard inputs and bridge validated 8/32/44-byte UDP packets. */
class UDPNode : public rclcpp::Node
{
public:
    /** @brief Bind two UDP channels before creating the standalone ROS interface. */
    UDPNode() : Node("udp_node")
    {
        const auto local          = declare_parameter<std::string>("local_address", JETSON_ADDRESS);
        const auto peer           = declare_parameter<std::string>("peer_address", BOARD_ADDRESS);
        const auto command_port   = port_parameter("command_port", COMMAND_PORT);
        const auto operation_port = port_parameter("operation_port", OPERATION_PORT);
        if (command_port == operation_port) {
            throw std::invalid_argument("command_port and operation_port must differ");
        }
        command_peer_  = {peer, command_port};
        feedback_peer_ = {peer, operation_port};
        command_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local, command_port}, command_peer_);
        operation_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local, operation_port}, feedback_peer_);

        // Preserve ce821b9's standard feedback topics and their units.
        sequence_pub_     = create_publisher<UInt8>("/robot/feedback/sequence", 10);
        emergency_pub_    = create_publisher<Bool>("/robot/feedback/emergency_stop", 10);
        over_current_pub_ = create_publisher<Bool>("/robot/feedback/over_current", 10);
        wheel_pub_ = create_publisher<Float32Array>("/robot/feedback/wheel_angular_velocity", 10);
        constexpr std::array<const char*, 7> topics{
            "drive_battery_voltage",
            "drive_current",
            "belt_launcher_target",
            "belt_launcher_release",
            "belt_launcher_velocity",
            "loading_belt_angle",
            "bucket_arm_hight"};
        for (std::size_t measurement_index = 0; measurement_index < topics.size();
             ++measurement_index) {
            measurement_pubs_[measurement_index] = create_publisher<Float32>(
                std::string("/robot/feedback/") + topics[measurement_index], 10
            );
        }
        request_pub_ = create_publisher<UInt8Array>("/robot/request", 10);
        status_pub_  = create_publisher<UInt8>("/robot/feedback/navigation_status", 10);
        create_inputs();
        receive_timer_ = create_wall_timer(10ms, [this]() { receive_packets(); });
        send_timer_    = create_wall_timer(100ms, [this]() { transmit(); });
        RCLCPP_INFO(
            get_logger(),
            "Standard UDP bridge: %s -> %s, command=%u operation=%u",
            local.c_str(),
            peer.c_str(),
            command_port,
            operation_port
        );
    }

private:
    /** @brief Reject invalid configuration instead of silently changing the endpoint. */
    uint16_t port_parameter(const std::string& name, uint16_t initial)
    {
        const auto value = declare_parameter<int64_t>(name, initial);
        if (value < 1 || value > 65535) {
            throw std::invalid_argument(name + " must be in [1,65535]");
        }
        return static_cast<uint16_t>(value);
    }

    /** @brief Retain a volatile subscription to the most recent independent input. */
    template <typename Message, typename Callback>
    void subscribe(const std::string& name, Callback callback)
    {
        subscriptions_.push_back(create_subscription<Message>(
            "/robot/command/" + name, rclcpp::QoS(1).durability_volatile(), callback
        ));
    }

    /** @brief Validate a floating setpoint and leave its previous value unchanged on rejection. */
    void set_float(float value, float& destination)
    {
        if (!std::isfinite(value)) {
            reject_input();
            return;
        }
        destination = value;
    }

    /** @brief Set up only standard ROS messages; no aggregate operation or heartbeat is used. */
    void create_inputs()
    {
        subscribe<Bool>("localization_valid", [this](Bool::ConstSharedPtr message) {
            const auto previous = state_.status();
            state_.observe_localization(message->data);
            state_changed(previous);
        });
        subscribe<UInt8>("navigation_status", [this](UInt8::ConstSharedPtr message) {
            if (message->data > static_cast<uint8_t>(NavigationStatus::Fail)) {
                reject_input();
                return;
            }
            const auto previous = state_.status();
            state_.observe_navigation(static_cast<NavigationStatus>(message->data));
            state_changed(previous);
        });
        subscribe<Twist>("cmd_vel", [this](Twist::ConstSharedPtr message) {
            for (double value : {message->linear.x, message->linear.y, message->angular.z}) {
                if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
                    reject_input();
                    return;
                }
            }
            if (state_.motion_allowed()) {
                operation_.velocity_x_m_s     = static_cast<float>(message->linear.x);
                operation_.velocity_y_m_s     = static_cast<float>(message->linear.y);
                operation_.velocity_yaw_rad_s = static_cast<float>(message->angular.z);
            }
        });
        subscribe<UInt8>("target_id", [this](UInt8::ConstSharedPtr message) {
            if (message->data > 7) {
                reject_input();
                return;
            }
            operation_.target_id = message->data;
        });
        subscribe<Float32>("belt_launcher_speed", [this](Float32::ConstSharedPtr message) {
            if (!std::isfinite(message->data)) {
                reject_input();
                return;
            }
            if (state_.ready()) {
                operation_.belt_launcher_speed_m_s = message->data;
            }
        });
        subscribe<Float32>("bucket_arm_height", [this](Float32::ConstSharedPtr message) {
            set_float(message->data, operation_.bucket_arm_height_m);
        });
        subscribe<Bool>("bucket_arm_width_extract", [this](Bool::ConstSharedPtr message) {
            operation_.bucket_arm_width_extract = message->data;
        });
        subscribe<Bool>("bucket_arm_hold", [this](Bool::ConstSharedPtr message) {
            operation_.bucket_arm_hold = message->data;
        });
        subscribe<Bool>("can_move", [this](Bool::ConstSharedPtr message) {
            operation_.can_move = message->data;
        });
        subscribe<Int8Array>("angles_deg", [this](Int8Array::ConstSharedPtr message) {
            if (message->data.size() != operation_.angles_deg.size()) {
                reject_input();
                return;
            }
            for (auto angle : message->data) {
                if (angle < -20 || angle > 20) {
                    reject_input();
                    return;
                }
            }
            std::copy(message->data.begin(), message->data.end(), operation_.angles_deg.begin());
        });
    }

    /** @brief Report rejected input without inventing a localization/navigation failure. */
    void reject_input()
    {
        RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Rejected invalid ROS input");
    }

    /** @brief Discard chassis commands instead of caching them for a future phase. */
    void clear_velocity()
    {
        operation_.velocity_x_m_s     = 0;
        operation_.velocity_y_m_s     = 0;
        operation_.velocity_yaw_rad_s = 0;
    }

    /** @brief Clear stopped output and immediately report entry into explicit Fail. */
    void state_changed(NavigationStatus previous)
    {
        if (!state_.motion_allowed()) {
            clear_velocity();
        }
        if (!state_.ready()) {
            operation_.belt_launcher_speed_m_s = 0;
        }
        if (previous != NavigationStatus::Fail && state_.status() == NavigationStatus::Fail) {
            RCLCPP_WARN(
                get_logger(), "NavigationStatus -> Fail: explicit localization/navigation failure"
            );
            transmit();
        }
    }

    /** @brief Publish the data member of a standard message. */
    template <typename Message, typename Value>
    void publish_value(
        const std::shared_ptr<rclcpp::Publisher<Message>>& publisher, const Value& value
    )
    {
        Message message;
        message.data = value;
        publisher->publish(message);
    }

    /** @brief Publish one accepted feedback snapshot as ce821b9's individual standard topics. */
    void publish_feedback(const wire::Feedback& feedback)
    {
        publish_value(sequence_pub_, feedback.sequence);
        publish_value(emergency_pub_, feedback.emergency_stop_enabled);
        publish_value(over_current_pub_, feedback.over_current);
        publish_value(
            wheel_pub_,
            std::vector<float>(
                feedback.wheel_angular_velocity_rad_s.begin(),
                feedback.wheel_angular_velocity_rad_s.end()
            )
        );
        const std::array<float, 7> values{
            feedback.drive_battery_voltage_v,
            feedback.drive_current_a,
            feedback.belt_launcher_target_velocity_m_s,
            feedback.last_belt_launcher_release_velocity_m_s,
            feedback.belt_launcher_velocity_m_s,
            feedback.loading_belt_angle_rad,
            feedback.bucket_arm_height_m};
        for (std::size_t measurement_index = 0; measurement_index < values.size();
             ++measurement_index) {
            publish_value(measurement_pubs_[measurement_index], values[measurement_index]);
        }
    }

    /** @brief Drain bounded packet batches so a flood cannot starve ROS callbacks or sending. */
    void receive_packets()
    {
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, wire::COMMAND_SIZE> bytes{};
            const auto result = command_channel_->receive(bytes);
            if (result.status == DatagramResult::Status::NO_DATA) {
                break;
            }
            wire::Command command{};
            if (result.status == DatagramResult::Status::ERROR) {
                RCLCPP_ERROR_THROTTLE(get_logger(), log_clock_, 2000, "Command UDP receive failed");
                break;
            }
            if (result.source != command_peer_ || result.truncated || result.size != bytes.size() ||
                !wire::decode_command(bytes, command)) {
                continue;
            }
            // Only a real non-Move-to-Move edge can clear an already healthy Fail latch.
            if (state_.observe_command(command.navigation_command)) {
                clear_velocity();
                operation_.belt_launcher_speed_m_s = 0;
            }
            publish_value(
                request_pub_,
                std::vector<uint8_t>{
                    command.sequence,
                    static_cast<uint8_t>(command.shutdown),
                    static_cast<uint8_t>(command.localization),
                    static_cast<uint8_t>(command.logging),
                    command.target_id,
                    command.navigation_command,
                    static_cast<uint8_t>(command.cloth_collect)}
            );
        }
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, wire::FEEDBACK_SIZE> bytes{};
            const auto result = operation_channel_->receive(bytes);
            if (result.status == DatagramResult::Status::NO_DATA) {
                break;
            }
            if (result.status == DatagramResult::Status::ERROR) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), log_clock_, 2000, "Feedback UDP receive failed"
                );
                break;
            }
            wire::Feedback feedback{};
            if (result.source == feedback_peer_ && !result.truncated &&
                result.size == bytes.size() && wire::decode_feedback(bytes, feedback)) {
                publish_feedback(feedback);
            }
        }
    }

    /** @brief Send the effective state in byte 3 and LED-only can_move in byte 1. */
    void transmit()
    {
        operation_.navigation_status = static_cast<uint8_t>(state_.status());
        if (!state_.motion_allowed()) {
            clear_velocity();
        }
        if (!state_.ready()) {
            operation_.belt_launcher_speed_m_s = 0;
        }
        std::array<uint8_t, wire::OPERATION_SIZE> bytes{};
        if (!wire::encode_operation(operation_, bytes) || !operation_channel_->send(bytes)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), log_clock_, 2000, "Operation UDP send failed");
        }
        publish_value(status_pub_, operation_.navigation_status);
    }

    NavigationState state_;
    wire::Operation operation_{};
    std::unique_ptr<UDPChannel> command_channel_, operation_channel_;
    Endpoint command_peer_, feedback_peer_;
    rclcpp::Clock log_clock_{RCL_STEADY_TIME};
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Publisher<UInt8>::SharedPtr sequence_pub_, status_pub_;
    rclcpp::Publisher<Bool>::SharedPtr emergency_pub_, over_current_pub_;
    rclcpp::Publisher<Float32Array>::SharedPtr wheel_pub_;
    std::array<rclcpp::Publisher<Float32>::SharedPtr, 7> measurement_pubs_;
    rclcpp::Publisher<UInt8Array>::SharedPtr request_pub_;
    rclcpp::TimerBase::SharedPtr receive_timer_, send_timer_;
};
}  // namespace robot_port

/** @brief Run the standalone bridge; invalid endpoint configuration exits visibly. */
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
