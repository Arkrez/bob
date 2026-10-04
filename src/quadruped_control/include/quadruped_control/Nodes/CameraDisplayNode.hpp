#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

class CameraDisplayNode : public rclcpp::Node
{
public:
    CameraDisplayNode();

private:
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );

    void toggleDisplayCallback(
        const std_msgs::msg::Bool::SharedPtr msg
    );

    bool displayEnabled_ = false;

    rclcpp::Subscription<
        sensor_msgs::msg::Image
    >::SharedPtr imageSubscription_;

    rclcpp::Subscription<
        std_msgs::msg::Bool
    >::SharedPtr displaySubscription_;
};