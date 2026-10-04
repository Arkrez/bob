#include "quadruped_control/Nodes/CameraDisplayNode.hpp"

#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

CameraDisplayNode::CameraDisplayNode()
    : Node("camera_display_node")
{
    imageSubscription_ =
        create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(
                &CameraDisplayNode::imageCallback,
                this,
                std::placeholders::_1
            )
        );

    displaySubscription_ =
        create_subscription<std_msgs::msg::Bool>(
            "/toggle_camera_display",
            10,
            std::bind(
                &CameraDisplayNode::toggleDisplayCallback,
                this,
                std::placeholders::_1
            )
        );

    RCLCPP_INFO(
        get_logger(),
        "Camera display node started."
    );
}

void CameraDisplayNode::toggleDisplayCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    displayEnabled_ = msg->data;

    if (displayEnabled_)
    {
        RCLCPP_INFO(
            get_logger(),
            "Camera display ON"
        );
    }
    else
    {
        RCLCPP_INFO(
            get_logger(),
            "Camera display OFF"
        );

        cv::destroyWindow(
            "Quadruped Camera"
        );

        cv::waitKey(1);
    }
}
void CameraDisplayNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!displayEnabled_)
    {
        return;
    }

    try
    {
        RCLCPP_INFO_ONCE(
            get_logger(),
            "Camera encoding: %s | %ux%u | step=%u",
            msg->encoding.c_str(),
            msg->width,
            msg->height,
            msg->step
        );

        cv::Mat displayImage;

        if (msg->encoding == "bgra8")
        {
            auto cvImage =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::BGRA8
                );

            cv::cvtColor(
                cvImage->image,
                displayImage,
                cv::COLOR_BGRA2BGR
            );
        }
        else if (msg->encoding == "bgr8")
        {
            auto cvImage =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::BGR8
                );

            displayImage = cvImage->image;
        }
        else if (msg->encoding == "rgb8")
        {
            auto cvImage =
                cv_bridge::toCvShare(
                    msg,
                    sensor_msgs::image_encodings::RGB8
                );

            cv::cvtColor(
                cvImage->image,
                displayImage,
                cv::COLOR_RGB2BGR
            );
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Unsupported image encoding: %s",
                msg->encoding.c_str()
            );

            return;
        }

        cv::imshow(
            "Quadruped Camera",
            displayImage
        );

        cv::waitKey(1);
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "OpenCV error: %s",
            e.what()
        );
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "cv_bridge error: %s",
            e.what()
        );
    }
}