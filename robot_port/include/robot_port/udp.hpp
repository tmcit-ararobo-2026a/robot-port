#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "robot_port/simple_udp.hpp"
#include "robot_port/robot_data_config.hpp"
#include "robot_port/ethernet_config.hpp"

class UDP : public rclcpp::Node
{
private:
    operation_data_t operation_data_;
    std::shared_ptr<SimpleUDP> simple_udp_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
public:
    UDP();
    ~UDP();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};