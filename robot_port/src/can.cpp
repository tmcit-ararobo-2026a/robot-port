#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

// 正しいパスでインクルード
#include "robot_port/linux_can_interface.hpp"

using namespace std::chrono_literals;

class LinuxCanNode : public rclcpp::Node
{
public:
    // 型名を gn10_can::drivers::LinuxCANDriver に修正
    LinuxCanNode() : Node("can_node"), can_intf_("can0")
    {
        if (!can_intf_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface!");
            throw std::runtime_error("CAN Open Failed");
        }

        pub_can_rx_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_rx", 10);

        auto cb_group_tx = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt     = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_tx;

        sub_can_tx_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "can_tx",
            10,
            std::bind(&LinuxCanNode::tx_callback, this, std::placeholders::_1),
            sub_opt
        );

        rx_thread_ = std::thread(&LinuxCanNode::rx_worker, this);
        RCLCPP_INFO(this->get_logger(), "CAN Node started using LinuxCANDriver.");
    }

    ~LinuxCanNode()
    {
        if (rx_thread_.joinable()) rx_thread_.join();
    }

private:
    void tx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        // gn10_can::FDCANFrame を作成して送信
        gn10_can::FDCANFrame frame;
        frame.id = 0x123; // 運用に合わせて変更してください
        frame.is_extended = false;
        frame.set_data(msg->data.data(), msg->data.size());

        if (!can_intf_.send(frame)) {
            RCLCPP_WARN(this->get_logger(), "CAN Send failed");
        }
    }

    void rx_worker()
    {
        int can_fd = can_intf_.get_socket_fd();

        while (rclcpp::ok()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(can_fd, &fds);

            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 100000;

            int ret = select(can_fd + 1, &fds, NULL, NULL, &tv);

            if (ret > 0 && FD_ISSET(can_fd, &fds)) {
                gn10_can::FDCANFrame rx_frame;
                // receive() は bool を返す仕様に合わせて修正
                while (can_intf_.receive(rx_frame)) {
                    auto msg = std_msgs::msg::UInt8MultiArray();
                    // dlc 分だけデータを assign
                    msg.data.assign(rx_frame.data.begin(), rx_frame.data.begin() + rx_frame.dlc);
                    pub_can_rx_->publish(msg);
                }
            }
        }
    }

    // ここを正しいクラス名に変更
    gn10_can::drivers::LinuxCANDriver can_intf_;
    
    std::thread rx_thread_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_can_rx_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_can_tx_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinuxCanNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}