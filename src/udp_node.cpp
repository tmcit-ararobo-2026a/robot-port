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
using Int8Array    = std_msgs::msg::Int8MultiArray;
using UInt8        = std_msgs::msg::UInt8;
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
        operation_.value.header                = robot_config::header::operation;
        operation_.value.move_bucket_angle_yaw = INT8_MIN;
        operation_.value.bucket1_angle_yaw     = INT8_MIN;
        operation_.value.bucket2_angle_yaw     = INT8_MIN;
        operation_.value.bucket3_angle_yaw     = INT8_MIN;
        operation_.value.flag_angle_yaw        = INT8_MIN;
        operation_.value.desk_angle_yaw        = INT8_MIN;

        // パラメータ
        const auto default_local   = ip_address(robot_config::ip::pc_robot);
        const auto default_peer    = ip_address(robot_config::ip::mainboard);
        auto local_address         = declare_parameter<std::string>("local_address", default_local);
        auto peer_address          = declare_parameter<std::string>("peer_address", default_peer);
        auto operation_freq        = declare_parameter<int>("operation_freq", 20);
        auto receive_timer_freq    = declare_parameter<int>("receive_timer_freq", 100);
        localization_timeout_      = timeout_parameter("localization_timeout_ms", 1000);
        navigation_status_timeout_ = timeout_parameter("navigation_status_timeout_ms", 1000);
        // アドレスが空白ならデフォルト値を用いる
        if (local_address.empty()) {
            local_address = default_local;
        }
        if (peer_address.empty()) {
            peer_address = default_peer;
        }
        // ポートは基本固定
        const auto operation_port = port_parameter("operation_port", robot_config::port::operation);
        // UDPのソケット作成とbind
        feedback_peer_ = {peer_address, operation_port};
        operation_channel_ =
            std::make_unique<UDPChannel>(Endpoint{local_address, operation_port}, feedback_peer_);

        // ROS2のインターフェイス
        emergency_pub_ = create_publisher<Bool>("/robot/feedback/emergency_stop", 10);
        wheel_pub_ = create_publisher<Float32Array>("/robot/feedback/wheel_angular_velocity", 10);
        constexpr std::array<const char*, 7> topics{
            "drive_battery_voltage",
            "drive_current",
            "belt_launcher/target",
            "belt_launcher/command",
            "belt_launcher/velocity",
            "belt_launcher/voltage",
            "belt_launcher/current"};
        for (std::size_t measurement_index = 0; measurement_index < topics.size();
             ++measurement_index) {
            measurement_pubs_[measurement_index] = create_publisher<Float32>(
                std::string("/robot/feedback/") + topics[measurement_index], 10
            );
        }
        // 結局のステータス
        status_pub_ = create_publisher<UInt8>("/robot/feedback/navigation_status", 10);
        // メイン基板からの司令
        navigation_command_pub_ = create_publisher<UInt8>("/robot/navigation_command", 10);
        target_id_              = create_publisher<UInt8>("/robot/target_id", 10);
        // 射出カウンタトピック (メイン基板のfeedback.shot_countを配信)
        shot_count_pub_ = create_publisher<UInt8>("/robot/feedback/shot_count", 10);

        // Operationデータ送信用にROS2の受信
        create_inputs();
        // 送受信タイマーをパラメータで指定した周波数で作成
        receive_timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / receive_timer_freq), [this]() { receive_packets(); }
        );
        send_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / operation_freq), [this]() {
            transmit();
        });
        RCLCPP_INFO(
            get_logger(),
            "Standard UDP bridge: %s -> %s, operation=%u",
            local_address.c_str(),
            peer_address.c_str(),
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

    /** @brief Declare a positive timeout parameter in milliseconds. */
    std::chrono::milliseconds timeout_parameter(const std::string& name, int initial)
    {
        const auto value = declare_parameter<int>(name, initial);
        if (value < 1) {
            throw std::invalid_argument(name + " must be greater than zero");
        }
        return std::chrono::milliseconds(value);
    }

    template <typename Message, typename Callback>
    void subscribe(const std::string& name, Callback callback)
    {
        subscriptions_.push_back(create_subscription<Message>(
            "/robot/operation/" + name, rclcpp::QoS(1).durability_volatile(), callback
        ));
    }

    void create_inputs()
    {
        subscribe<Bool>("localization_success", [this](Bool::ConstSharedPtr message) {
            operation_.value.localization_success = static_cast<bool>(message->data);
            last_localization_success_time_       = std::chrono::steady_clock::now();
        });
        subscribe<UInt8>("navigation_status", [this](UInt8::ConstSharedPtr message) {
            if (message->data > static_cast<uint8_t>(robot_config::NavigationStatus::Fail)) {
                reject_input();
                return;
            }
            operation_.value.navigation_status =
                static_cast<robot_config::NavigationStatus>(message->data);
            last_navigation_status_time_ = std::chrono::steady_clock::now();
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
        subscribe<Int8Array>("target_angles", [this](Int8Array::ConstSharedPtr message) {
            if (message->data.size() != 6) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    log_clock_,
                    2000,
                    "target_angles must have 6 elements, received %zu",
                    message->data.size()
                );
                return;
            }
            operation_.value.move_bucket_angle_yaw = message->data[0];
            operation_.value.bucket1_angle_yaw     = message->data[1];
            operation_.value.bucket2_angle_yaw     = message->data[2];
            operation_.value.bucket3_angle_yaw     = message->data[3];
            operation_.value.flag_angle_yaw        = message->data[4];
            operation_.value.desk_angle_yaw        = message->data[5];
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
            feedback.belt_launcher_command_velocity,
            feedback.belt_launcher_velocity,
            feedback.belt_launcher_voltage,
            feedback.belt_launcher_current};
        for (std::size_t measurement_index = 0; measurement_index < values.size();
             ++measurement_index) {
            publish_value(measurement_pubs_[measurement_index], values[measurement_index]);
        }
        publish_value(
            navigation_command_pub_, static_cast<uint8_t>(feedback.command.navigation_command)
        );
        publish_value(target_id_, static_cast<uint8_t>(feedback.command.target_id));
        publish_value(shot_count_pub_, feedback.shot_count);
    }

    void receive_packets()
    {
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

    /** @brief Apply safe input values after localization or navigation input expires. */
    void update_input_timeouts()
    {
        const auto current_time = std::chrono::steady_clock::now();
        if (current_time - last_localization_success_time_ >= localization_timeout_) {
            operation_.value.localization_success = false;
            RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Localization health timeout");
        }
        if (current_time - last_navigation_status_time_ >= navigation_status_timeout_) {
            operation_.value.navigation_status = robot_config::NavigationStatus::Fail;
            RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Navigation health timeout");
        }
    }

    void transmit()
    {
        update_input_timeouts();
        if (!operation_channel_->send(operation_.binary)) {
            RCLCPP_ERROR_THROTTLE(get_logger(), log_clock_, 2000, "Operation UDP send failed");
        }
    }

    robot_config::operation_u operation_{};
    std::unique_ptr<UDPChannel> operation_channel_;
    Endpoint feedback_peer_;
    rclcpp::Clock log_clock_{RCL_STEADY_TIME};
    std::chrono::milliseconds localization_timeout_{1000};
    std::chrono::milliseconds navigation_status_timeout_{1000};
    std::chrono::steady_clock::time_point last_localization_success_time_{
        std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point last_navigation_status_time_{
        std::chrono::steady_clock::now()};
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Publisher<UInt8>::SharedPtr sequence_pub_, status_pub_;
    rclcpp::Publisher<Bool>::SharedPtr emergency_pub_, over_current_pub_;
    rclcpp::Publisher<Float32Array>::SharedPtr wheel_pub_;
    rclcpp::Publisher<UInt8>::SharedPtr navigation_command_pub_;
    rclcpp::Publisher<UInt8>::SharedPtr target_id_;
    rclcpp::Publisher<UInt8>::SharedPtr shot_count_pub_;
    std::array<rclcpp::Publisher<Float32>::SharedPtr, 7> measurement_pubs_;
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
