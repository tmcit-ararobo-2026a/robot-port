#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>  // 汎用的な例として

#include "robot_port/linux_can_interface.hpp"

using namespace std::chrono_literals;

class LinuxCanNode : public rclcpp::Node
{
public:
    LinuxCanNode() : Node("can_node"), can_intf_("can0")
    {
        // 1. CANインターフェースのオープン
        if (!can_intf_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface!");
            throw std::runtime_error("CAN Open Failed");
        }

        // 2. パブリッシャの作成
        pub_can_rx_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_rx", 10);

        // 3. 送信用サブスクライバ（CallbackGroupで分離）
        auto cb_group_tx = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt     = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = cb_group_tx;

        sub_can_tx_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "can_tx",
            10,
            std::bind(&LinuxCanNode::tx_callback, this, std::placeholders::_1),
            sub_opt
        );

        // 4. 受信専用スレッドの開始 (WaitSetを使用)
        rx_thread_ = std::thread(&LinuxCanNode::rx_worker, this);
        RCLCPP_INFO(this->get_logger(), "CAN Node started with WaitSet RX thread.");
    }

    ~LinuxCanNode()
    {
        if (rx_thread_.joinable()) rx_thread_.join();
    }

private:
    // 送信処理（トピックを受け取ってCANへ）
    void tx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        // 例: ID 0x123 で送信
        if (!can_intf_.send(0x123, msg->data)) {
            RCLCPP_WARN(this->get_logger(), "CAN Send failed (Buffer full?)");
        }
    }

    // 受信ワーカー（WaitSetによるイベント駆動）
    void rx_worker()
    {
        // ROS 2の終了を検知するための条件変数
        auto stop_condition = std::make_shared<rclcpp::GuardCondition>(
            this->get_node_base_interface()->get_context()
        );

        // WaitSetの設定（GuardConditionを1つ監視するように設定）
        rclcpp::WaitSet wait_set;
        wait_set.add_guard_condition(stop_condition);

        // SocketCANのfd
        int can_fd = can_intf_.get_socket_fd();

        while (rclcpp::ok()) {
            // POSIXのpollを使用して、CANソケットとGuardConditionの両方を監視したいところですが
            // シンプルに「CANソケットの監視」と「ROSの終了確認」を両立させるため、
            // 以前の select() を少し改良した形で実装します。

            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(can_fd, &fds);

            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 100000;  // 100ms タイムアウト

            int ret = select(can_fd + 1, &fds, NULL, NULL, &tv);

            if (ret > 0 && FD_ISSET(can_fd, &fds)) {
                // データが届いた！
                while (auto frame = can_intf_.receive()) {
                    auto msg = std_msgs::msg::UInt8MultiArray();
                    msg.data.assign(frame->data, frame->data + frame->len);
                    pub_can_rx_->publish(msg);
                }
            }

            // タイムアウトまたは受信後に rclcpp::ok() をチェックしてループ
        }
    }

    LinuxCanInterface can_intf_;
    std::thread rx_thread_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_can_rx_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_can_tx_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // StaticExecutorを使用して、管理コストを最小化
    auto node = std::make_shared<LinuxCanNode>();
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}