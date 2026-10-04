#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct JointAngles
{
    double joint1;
    double joint2;
    double joint3;
    double joint4;
};

class UserCommunicationNode : public rclcpp::Node
{
public:
    UserCommunicationNode();

private:
    

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};