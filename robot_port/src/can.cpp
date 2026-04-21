#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
// ライブラリ群と定義のインクルード
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/robot_control_hub_client.hpp"
#include "robot_port/linux_can_interface.hpp"
#include "robot_port/robot_data_config.hpp"

using namespace std::chrono_literals;

class LinuxCanNode : public rclcpp::Node
{
public:
    LinuxCanNode()
        : Node("can_node"),
          can_driver_("can0"),
          can_bus_(can_driver_),
          // ID 0 の RobotControlHubClient をインスタンス化
          control_hub_client_(can_bus_, 0)
    {
        // 1. ドライバのオープン
        if (!can_driver_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface!");
            throw std::runtime_error("CAN Open Failed");
        }

        // 2. operation_data_t の初期設定（ヘッダーを固定）
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            command_.header   = operation_data_header;  // 0xAB36
            command_.vx       = 0.0f;
            command_.vy       = 0.0f;
            command_.omega    = 0.0f;
            command_.cross    = 0;
            command_.cricle   = 0;
            command_.square   = 0;
            command_.triangle = 0;
        }

        // 3.1 cmd_vel サブスクライバ
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LinuxCanNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        // 3.2 button サブスクライバ
        sub_button_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "button", 10, std::bind(&LinuxCanNode::button_callback, this, std::placeholders::_1)
        );

        // 4. 100Hz 送信タイマー (10ms間隔)
        timer_ = this->create_wall_timer(10ms, std::bind(&LinuxCanNode::timer_callback, this));

        // 5. 受信スレッド (Busのupdate用)
        rx_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                // Bus::update() で dispatch まで実行
                can_bus_.update();
                // CPU負荷を抑えるための微小スリープ
                std::this_thread::sleep_for(1ms);
            }
        });

        RCLCPP_INFO(
            this->get_logger(),
            "CAN Control Node started (100Hz Loop with header 0x%04X)",
            operation_data_header
        );
    }

    ~LinuxCanNode()
    {
        if (rx_thread_.joinable()) rx_thread_.join();
    }

private:
    // cmd_vel から operation_data_t へ変換
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        command_.vx    = msg->linear.x;
        command_.vy    = msg->linear.y;
        command_.omega = msg->angular.z;
    }

    void button_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        uint8_t command_data_;
        command_data_     = msg->data[0];
        command_.cross    = command_data_ & 1;  // cross
        command_.cricle   = (command_data_ >> 1) & 1;
        command_.triangle = (command_data_ >> 2) & 1;
        command_.square   = (command_data_ >> 3) & 1;
    }

    // 100Hz で実行される送信処理
    void timer_callback()
    {
        operation_data_t op;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            op = command_;
        }

        // RobotControlHubClient は内部で converter::pack (memcpy)
        // を使用するため、構造体をそのまま投げればOK
        control_hub_client_.send_command(op);
    }

    // メンバ変数
    gn10_can::drivers::LinuxCANDriver can_driver_;
    gn10_can::FDCANBus can_bus_;
    gn10_can::devices::RobotControlHubClient<operation_data_t, feedback_data_t> control_hub_client_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_button_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread rx_thread_;

    operation_data_t command_;
    std::mutex data_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinuxCanNode>());
    rclcpp::shutdown();
    return 0;
}