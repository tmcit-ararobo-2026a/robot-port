#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/robot_control_hub_client.hpp"
#include "robot_port/linux_fdcan_driver.hpp"
#include "robot_port/robot_config.hpp"

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
        if (!can_driver_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface!");
            throw std::runtime_error("CAN Open Failed");
        }

        command_.header = robot_config::header::operation;

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                command_.x_vel       = msg->linear.x;
                command_.y_vel       = msg->linear.y;
                command_.angular_vel = msg->angular.z;
            }
        );

        sub_belt_vel_ = this->create_subscription<std_msgs::msg::Float32>(
            "/belt/speed_ratio", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.belt_vel = msg->data;
            }
        );
        sub_belt_throw_ = this->create_subscription<std_msgs::msg::Bool>(
            "/belt/throw", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.belt_throw = msg->data;
            }
        );
        sub_belt_init = this->create_subscription<std_msgs::msg::Bool>(
            "/belt/init", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.belt_init = msg->data;
            }
        );

        sub_bucket_arm_hight_ = this->create_subscription<std_msgs::msg::Float32>(
            "/bucket/arm/hight", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.bucket_arm_hight = static_cast<uint8_t>(msg->data) * 100.0f;  // m -> cm
            }
        );
        sub_bucket_arm_hold_ = this->create_subscription<std_msgs::msg::Bool>(
            "/bucket/arm/hold", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.bucket_arm_hold = msg->data;
            }
        );

        sub_desk_pos_ = this->create_subscription<std_msgs::msg::Float32>(
            "/desk/arm/pos", 10, [this](std_msgs::msg::Float32::SharedPtr msg) {
                command_.desk_arm_pos = static_cast<uint8_t>(msg->data) * 100.0f;  // m -> cm
            }
        );
        sub_desk_arm_hold_ = this->create_subscription<std_msgs::msg::Bool>(
            "/desk/arm/hold", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.desk_arm_hold = msg->data;
            }
        );

        sub_loading_hook_phase_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/loading/hook/phase", 10, [this](std_msgs::msg::UInt8::SharedPtr msg) {
                command_.loading_hook_phase = static_cast<uint8_t>(msg->data);
            }
        );
        sub_shift_cloth_ = this->create_subscription<std_msgs::msg::Bool>(
            "/loading/shift_cloth", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.loading_shift_cloth = msg->data;
            }
        );

        sub_air_rauncher_for_flag_ = this->create_subscription<std_msgs::msg::Bool>(
            "/air_rauncher/for_flag", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.air_rauncher_for_flag = msg->data;
            }
        );
        sub_air_rauncher_for_desk_r_ = this->create_subscription<std_msgs::msg::Bool>(
            "/air_rauncher/for_desk_r", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.air_rauncher_for_desk_r = msg->data;
            }
        );
        sub_air_rauncher_for_desk_l_ = this->create_subscription<std_msgs::msg::Bool>(
            "/air_rauncher/for_desk_l", 10, [this](std_msgs::msg::Bool::SharedPtr msg) {
                command_.air_rauncher_for_desk_l = msg->data;
            }
        );

        // 4. 100Hz 送信タイマー (10ms間隔)
        timer_ = this->create_wall_timer(10ms, std::bind(&CANNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "CAN Control Node started (100Hz Loop with header 0x%04X)",
            robot_config::header::operation
        );
    }

private:
    void timer_callback()
    {
        control_hub_client_.send_command(command_);
    }

    gn10_can::drivers::LinuxFDCANDriver can_driver_;
    gn10_can::FDCANBus can_bus_;
    gn10_can::devices::RobotControlHubClient<robot_config::operation_t, robot_config::feedback_t>
        control_hub_client_;
    // 足回り
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    // ベルト直動機構
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_belt_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_belt_throw_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_belt_init;
    // バケツ用アーム
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_bucket_arm_hight_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bucket_arm_hold_;
    // 机からの装填機構
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_desk_pos_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_desk_arm_hold_;
    // 装填機構
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_loading_hook_phase_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_shift_cloth_;
    // エアシリンダー射出
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_air_rauncher_for_flag_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_air_rauncher_for_desk_r_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_air_rauncher_for_desk_l_;
    rclcpp::TimerBase::SharedPtr timer_;

    robot_config::operation_t command_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANNode>());
    rclcpp::shutdown();
    return 0;
}