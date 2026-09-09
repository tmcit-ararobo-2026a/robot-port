#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "robot_port/robot_config.hpp"
#include "robot_port/simple_udp.hpp"

using namespace std::chrono_literals;

class UdpNode : public rclcpp::Node
{
public:
    UdpNode() : Node("udp_node")
    {
        if (!udp_.initSocket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init UDP socket!");
            throw std::runtime_error("UDP Init Failed");
        }
        // 受信設定
        if (!udp_.bindSocket(robot_config::ip::pc_robot, robot_config::port::cmd)) {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to bind UDP socket on port %d!", robot_config::port::cmd
            );
            throw std::runtime_error("UDP Bind Failed");
        }
        // 送信先設定
        udp_.setTxAddr(robot_config::ip::pc_robot, robot_config::port::cmd);

        // --- Publishers 初期化 ---
        pub_sequence_ =
            this->create_publisher<std_msgs::msg::UInt8>("/robot/feedback/sequence", 10);

        // 電源周り
        pub_emergency_stop_ =
            this->create_publisher<std_msgs::msg::Bool>("/robot/feedback/emergency_stop", 10);
        pub_over_current_ =
            this->create_publisher<std_msgs::msg::Bool>("/robot/feedback/over_current", 10);
        pub_drive_battery_voltage_ = this->create_publisher<std_msgs::msg::Float32>(
            "/robot/feedback/drive_battery_voltage", 10
        );
        pub_logic_battery_voltages_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/robot/feedback/logic_battery_voltages", 10
        );
        pub_drive_current_ =
            this->create_publisher<std_msgs::msg::Float32>("/robot/feedback/drive_current", 10);

        // 各アクチュエータ
        pub_wheel_angular_velocity_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/robot/feedback/wheel_angular_velocity", 10
        );
        pub_belt_launcher_velocity_ = this->create_publisher<std_msgs::msg::Float32>(
            "/robot/feedback/belt_launcher_velocity", 10
        );
        pub_loading_belt_angle_ = this->create_publisher<std_msgs::msg::Float32>(
            "/robot/feedback/loading_belt_angle", 10
        );
        pub_bucket_arm_hight_ =
            this->create_publisher<std_msgs::msg::Float32>("/robot/feedback/bucket_arm_hight", 10);

        // 10ms (100Hz) 受信タイマー
        timer_ = this->create_wall_timer(10ms, std::bind(&UdpNode::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "UDP Receiver Node started (Listening on port %d, feedback size: %zu bytes)",
            robot_config::port::cmd,
            sizeof(robot_config::feedback_t)
        );
    }

    ~UdpNode()
    {
        udp_.closeSocket();
    }

private:
    void timer_callback()
    {
        robot_config::feedback_u rx_feedback;

        // パケットを受信
        int rx_len = udp_.recvPacket(rx_feedback.binary, sizeof(rx_feedback));

        // 届いたデータサイズとヘッダー(0x55)のチェック
        if (rx_len == sizeof(robot_config::feedback_u)) {
            if (rx_feedback.value.header == robot_config::header::feedback) {
                const auto& fb = rx_feedback.value;

                // シーケンス番号
                auto seq_msg = std_msgs::msg::UInt8();
                seq_msg.data = fb.sequence;
                pub_sequence_->publish(seq_msg);

                // 電源周り
                auto emg_msg = std_msgs::msg::Bool();
                emg_msg.data = fb.emergency_stop_enabled;
                pub_emergency_stop_->publish(emg_msg);

                auto oc_msg = std_msgs::msg::Bool();
                oc_msg.data = fb.over_current;
                pub_over_current_->publish(oc_msg);

                auto v_drive_msg = std_msgs::msg::Float32();
                v_drive_msg.data = fb.drive_battery_voltages;
                pub_drive_battery_voltage_->publish(v_drive_msg);

                auto v_logic_msg = std_msgs::msg::Float32MultiArray();
                v_logic_msg.data = {fb.logic_battery_voltages[0], fb.logic_battery_voltages[1]};
                pub_logic_battery_voltages_->publish(v_logic_msg);

                auto i_drive_msg = std_msgs::msg::Float32();
                i_drive_msg.data = fb.drive_current;
                pub_drive_current_->publish(i_drive_msg);

                // 各アクチュエータ
                auto wheel_msg = std_msgs::msg::Float32MultiArray();
                wheel_msg.data = {
                    fb.wheel_angular_velocity[0],
                    fb.wheel_angular_velocity[1],
                    fb.wheel_angular_velocity[2]
                };
                pub_wheel_angular_velocity_->publish(wheel_msg);

                auto belt_v_msg = std_msgs::msg::Float32();
                belt_v_msg.data = fb.belt_launcher_velocity;
                pub_belt_launcher_velocity_->publish(belt_v_msg);

                auto belt_a_msg = std_msgs::msg::Float32();
                belt_a_msg.data = fb.loading_belt_angle;
                pub_loading_belt_angle_->publish(belt_a_msg);

                auto arm_h_msg = std_msgs::msg::Float32();
                arm_h_msg.data = fb.bucket_arm_hight;
                pub_bucket_arm_hight_->publish(arm_h_msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Feedback - Seq: %u, V_Drive: %.2fV, I_Drive: %.2fA, Bucket: %.2fm",
                    fb.sequence,
                    fb.drive_battery_voltages,
                    fb.drive_current,
                    fb.bucket_arm_hight
                );
            }
        }
    }

    SimpleUDP udp_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_sequence_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emergency_stop_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_over_current_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_drive_battery_voltage_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_logic_battery_voltages_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_drive_current_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheel_angular_velocity_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_belt_launcher_velocity_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_loading_belt_angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_bucket_arm_hight_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpNode>());
    rclcpp::shutdown();
    return 0;
}
