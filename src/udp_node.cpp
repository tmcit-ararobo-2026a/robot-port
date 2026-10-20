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
#include <string>
#include <vector>

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

/** @brief Convert a four-octet robot configuration address to dotted decimal. */
std::string ip_address(const uint8_t address[4])
{
    return std::to_string(address[0]) + "." + std::to_string(address[1]) + "." +
           std::to_string(address[2]) + "." + std::to_string(address[3]);
}

class UDPNode : public rclcpp::Node
{
public:
    UDPNode() : Node("udp_node")
    {
        const auto default_local = ip_address(robot_config::ip::pc_robot);
        const auto default_peer  = ip_address(robot_config::ip::mainboard);
        auto local_address       = declare_parameter<std::string>("local_address", default_local);
        auto peer_address        = declare_parameter<std::string>("peer_address", default_peer);
        auto operation_freq      = declare_parameter<int>("operation_freq", 20);
        auto recieve_timer_freq  = declare_parameter<int>("receive_timer_freq", 100);
        if (local_address.empty()) {
            local_address = default_local;
        }
        if (peer_address.empty()) {
            peer_address = default_peer;
        }
        const auto command_port   = port_parameter("command_port", robot_config::port::command);
        const auto operation_port = port_parameter("operation_port", robot_config::port::operation);
        if (command_port == operation_port) {
            throw std::invalid_argument("command_port and operation_port must differ");
        }
        command_peer_  = {peer_address, command_port};
        feedback_peer_ = {peer_address, operation_port};
        command_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local_address, command_port}, command_peer_);
        operation_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local_address, operation_port}, feedback_peer_);

        emergency_pub_ = create_publisher<Bool>("/robot/feedback/emergency_stop", 10);
        wheel_pub_ = create_publisher<Float32Array>("/robot/feedback/wheel_angular_velocity", 10);
        constexpr std::array<const char*, 7> topics{
            "drive_battery_voltage",
            "drive_current",
            "belt_launcher/target",
            "belt_launcher/velocity",
            "belt_launcher/voltage",
            "belt_launcher/current"
        };
        for (std::size_t measurement_index = 0; measurement_index < topics.size();
             ++measurement_index) {
            measurement_pubs_[measurement_index] = create_publisher<Float32>(
                std::string("/robot/feedback/") + topics[measurement_index], 10
            );
        }
        request_pub_ = create_publisher<UInt8Array>("/robot/command", 10);
        status_pub_  = create_publisher<UInt8>("/robot/feedback/navigation_status", 10);
        create_inputs();
        receive_timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / recieve_timer_freq), [this]() { receive_packets(); }
        );
        send_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / operation_freq), [this]() {
            transmit();
        });
        RCLCPP_INFO(
            get_logger(),
            "Standard UDP bridge: %s -> %s, command=%u operation=%u",
            local_address.c_str(),
            peer_address.c_str(),
            command_port,
            operation_port
        );
    }

private:
    uint16_t port_parameter(const std::string& name, uint16_t initial)
    {
        const auto value = declare_parameter<int64_t>(name, initial);
        if (value < 1 || value > 65535) {
            throw std::invalid_argument(name + " must be in [1,65535]");
        }
        return static_cast<uint16_t>(value);
    }

    template <typename Message, typename Callback>
    void subscribe(const std::string& name, Callback callback)
    {
        subscriptions_.push_back(
            create_subscription<Message>(
                "/robot/operation/" + name, rclcpp::QoS(1).durability_volatile(), callback
            )
        );
    }

    void create_inputs()
    {
        subscribe<Bool>("fatal_error", [this](Bool::ConstSharedPtr message) {
            operation_.value.fatal_error = static_cast<bool>(message->data);
        });
        subscribe<UInt8>("navigation_status", [this](UInt8::ConstSharedPtr message) {
            if (message->data > static_cast<uint8_t>(robot_config::NavigationStatus::Fail)) {
                reject_input();
                return;
            }
            operation_.value.navigation_status =
                static_cast<robot_config::NavigationStatus>(message->data);
        });
        subscribe<Twist>("cmd_vel", [this](Twist::ConstSharedPtr message) {
            operation_.value.vel_x   = static_cast<float>(message->linear.x);
            operation_.value.vel_y   = static_cast<float>(message->linear.y);
            operation_.value.vel_yaw = static_cast<float>(message->angular.z);
        });
        subscribe<UInt8>("target_id", [this](UInt8::ConstSharedPtr message) {
            if (message->data > 7) {
                reject_input();
                return;
            }
            operation_.value.target_id = static_cast<robot_config::TargetId>(message->data);
        });
        subscribe<Float32>("belt_launcher_speed", [this](Float32::ConstSharedPtr message) {
            operation_.value.belt_launcher_speed = message->data;
        });
    }

    void reject_input()
    {
        operation_.value.navigation_status = robot_config::NavigationStatus::Fail;
        RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Rejected invalid ROS input");
    }

    template <typename Message, typename Value>
    void publish_value(
        const std::shared_ptr<rclcpp::Publisher<Message>>& publisher, const Value& value
    )
    {
        Message message;
        message.data = value;
        publisher->publish(message);
    }

    /**
     * @brief フィードバックをpublish
     *
     * @param feedback robot configのFeedbackで、内容が変わったらここも変えないといけない
     */
    void publish_feedback(const robot_config::feedback_t& feedback)
    {
        publish_value(sequence_pub_, feedback.sequence);
        publish_value(emergency_pub_, feedback.emergency_stop_enabled);
        publish_value(
            wheel_pub_,
            std::vector<float>(
                std::begin(feedback.wheel_angular_velocity),
                std::end(feedback.wheel_angular_velocity)
            )
        );
        const std::array<float, 7> values{
            feedback.drive_battery_voltages,
            feedback.drive_current,
            feedback.belt_launcher_target_velocity,
            feedback.belt_launcher_velocity,
            feedback.belt_launcher_voltage,
            feedback.belt_launcher_current
        };
        for (std::size_t measurement_index = 0; measurement_index < values.size();
             ++measurement_index) {
            publish_value(measurement_pubs_[measurement_index], values[measurement_index]);
        }
    }

    void receive_packets()
    {
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, sizeof(robot_config::command_u)> bytes{};
            const auto result = command_channel_->receive(bytes);
            if (result.status == DatagramResult::Status::NO_DATA) {
                break;
            }
            robot_config::command_u command{};
            if (result.status == DatagramResult::Status::ERROR) {
                RCLCPP_ERROR_THROTTLE(get_logger(), log_clock_, 2000, "Command UDP receive failed");
                break;
            }
            if (result.source == command_peer_ && !result.truncated &&
                result.size == bytes.size()) {
                memcpy(command.binary, bytes.data(), sizeof(robot_config::command_u));
            }
        }
        for (int packet_index = 0; packet_index < 32; ++packet_index) {
            std::array<uint8_t, sizeof(robot_config::feedback_u)> bytes{};
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
            robot_config::feedback_u feedback{};
            if (result.source == feedback_peer_ && !result.truncated &&
                result.size == bytes.size()) {
                memcpy(feedback.binary, bytes.data(), sizeof(robot_config::feedback_u));
                publish_feedback(feedback.value);
            }
        }
    }

    void transmit()
    {
        if (!operation_channel_->send(operation_.binary)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), log_clock_, 2000, "Operation UDP send failed");
        }
    }

    robot_config::operation_u operation_{};
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
