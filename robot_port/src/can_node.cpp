#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
// ライブラリ群と定義のインクルード
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/robot_control_hub_client.hpp"
#include "robot_port/linux_can_interface.hpp"
#include "robot_port/robot_data_config.hpp"

using namespace std::chrono_literals;

class CANNode : public rclcpp::Node
{
public:
    CANNode()
        : Node("can_node"),
          can_driver_("can0"),
          can_bus_(can_driver_),
          control_hub_client_(can_bus_, 0)
    {
        // 1. ドライバのオープン
        if (!can_driver_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface!");
            throw std::runtime_error("CAN Open Failed");
        }

        // 2. operation_data_t の初期設定（ヘッダーを固定）
        {
            command_.header           = operation_data_header;  // 0xAB36
            command_.wheel_front      = 0.0f;
            command_.wheel_back_left  = 0.0f;
            command_.wheel_back_right = 0.0f;
            command_.belt_velocity    = 0.0f;
            command_.arm_horizontal   = 0.0f;
            command_.arm_vertical     = 0.0f;
            command_.air_throw        = 0.0f;
            command_.arm_hold         = false;
            command_.belt_throw       = false;
            command_.collect          = false;
        }

        // 3. ROS2 サブスクライバーの設定
        sub_wheel_front_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel/front", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.wheel_front = msg->data;
            }
        );
        sub_wheel_back_left_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel/back_left", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.wheel_back_left = msg->data;
            }
        );
        sub_wheel_back_right_ = this->create_subscription<std_msgs::msg::Float32>(
            "/wheel/back_right", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.wheel_back_right = msg->data;
            }
        );
        sub_belt_velocity_ = this->create_subscription<std_msgs::msg::Float32>(
            "/belt/velocity", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.belt_velocity = msg->data;
            }
        );
        sub_belt_throw_ = this->create_subscription<std_msgs::msg::Bool>(
            "/belt/throw", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.belt_throw = msg->data;
            }
        );
        sub_collect_ = this->create_subscription<std_msgs::msg::Bool>(
            "/collect", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.collect = msg->data;
            }
        );
        sub_air_throw_ = this->create_subscription<std_msgs::msg::Float32>(
            "/air/throw", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.air_throw = msg->data
            }
        );

        // 4. 100Hz 送信タイマー (10ms間隔)
        timer_ = this->create_wall_timer(10ms, std::bind(&CANNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "CAN Control Node started (100Hz Loop with header 0x%04X)",
            operation_data_header
        );
    }

private:
    // 100Hz で実行される送信処理
    void timer_callback()
    {
        control_hub_client_.send_command(command_);
    }

    // メンバ変数
    gn10_can::drivers::LinuxCANDriver can_driver_;
    gn10_can::FDCANBus can_bus_;
    gn10_can::devices::RobotControlHubClient<operation_data_t, feedback_data_t> control_hub_client_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_wheel_front_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_wheel_back_left_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_wheel_back_right_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_belt_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_air_throw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_belt_throw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collect_;
    rclcpp::TimerBase::SharedPtr timer_;

    operation_data_t command_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANNode>());
    rclcpp::shutdown();
    return 0;
}