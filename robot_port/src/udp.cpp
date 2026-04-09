#include "robot_port/udp.hpp"

UDP::UDP() : Node("udp_node")
{
    simple_udp_ = std::make_shared<SimpleUDP>();
    simple_udp_->initSocket();
    simple_udp_->setTxAddr(ethernet_config::main_board::ip,
                   ethernet_config::main_board::port_operation);
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&UDP::cmd_vel_callback, this, std::placeholders::_1));
}

UDP::~UDP()
{
    simple_udp_->closeSocket();
}

void UDP::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    operation_data_.header = operation_data_header;
    operation_data_.vx = msg->linear.x;
    operation_data_.vy = msg->linear.y;
    operation_data_.omega = msg->angular.z;
    simple_udp_->sendPacket(reinterpret_cast<uint8_t*>(&operation_data_), sizeof(operation_data_));

    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear_x=%.2f, angular_z=%.2f",
                msg->linear.x, msg->angular.z);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDP>());
    rclcpp::shutdown();
    return 0;
}
