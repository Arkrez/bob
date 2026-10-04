#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

class Servo2040BridgeNode : public rclcpp::Node
{
public:
    Servo2040BridgeNode();
    ~Servo2040BridgeNode() override;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        subscription_;

    // Publishes:
    //   OK,<original command payload>
    //   FAIL,<original command payload>
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        status_publisher_;

    rclcpp::TimerBase::SharedPtr
        reconnect_timer_;

    std::string port_;
    int baud_ = 115200;
    int serial_fd_ = -1;

    bool openSerial();
    void closeSerial();

    bool writeCommand(
        const std::string& payload
    );

    void publishStatus(
        bool success,
        const std::string& payload
    );
};
