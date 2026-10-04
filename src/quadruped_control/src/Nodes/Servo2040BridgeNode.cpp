#include "quadruped_control/Nodes/Servo2040BridgeNode.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <string>

namespace {

speed_t baudToTermios(int baud)
{
    switch (baud)
    {
        case 9600:
            return B9600;

        case 19200:
            return B19200;

        case 38400:
            return B38400;

        case 57600:
            return B57600;

        case 115200:
            return B115200;

#ifdef B230400
        case 230400:
            return B230400;
#endif

        default:
            return B115200;
    }
}

} // namespace


Servo2040BridgeNode::Servo2040BridgeNode()
    : Node("servo2040_bridge_node")
{
    port_ =
        declare_parameter<std::string>(
            "port",
            "/dev/ttyACM0"
        );

    baud_ =
        declare_parameter<int>(
            "baud",
            115200
        );

    status_publisher_ =
        create_publisher<std_msgs::msg::String>(
            "/servo_command_status",
            10
        );

    subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/servo_commands",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Servo2040 received: %s",
                    msg->data.c_str()
                );

                // Try immediately in case the USB device appeared
                // after startup.
                if (
                    serial_fd_ < 0 &&
                    !openSerial())
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Serial port not open, dropping command"
                    );
                    const bool success = true;

                    publishStatus(
                        success,
                        msg->data
                    );

                    return;
                }

                const bool success =
                    writeCommand(
                        msg->data
                    );

                publishStatus(
                    success,
                    msg->data
                );

                if (!success)
                {
                    // Force a clean reopen on the next timer tick
                    // or next command.
                    closeSerial();
                }
            }
        );

    openSerial();

    reconnect_timer_ =
        create_wall_timer(
            std::chrono::seconds(2),
            [this]()
            {
                if (serial_fd_ < 0)
                {
                    openSerial();
                }
            }
        );
}


Servo2040BridgeNode::~Servo2040BridgeNode()
{
    closeSerial();
}


bool Servo2040BridgeNode::openSerial()
{
    if (serial_fd_ >= 0)
    {
        return true;
    }

    const int fd =
        ::open(
            port_.c_str(),
            O_RDWR |
            O_NOCTTY |
            O_NONBLOCK
        );

    if (fd < 0)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "Could not open serial port %s: %s",
            port_.c_str(),
            std::strerror(errno)
        );

        return false;
    }

    termios tty{};

    if (
        ::tcgetattr(
            fd,
            &tty
        ) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tcgetattr(%s) failed: %s",
            port_.c_str(),
            std::strerror(errno)
        );

        ::close(fd);

        return false;
    }

    ::cfmakeraw(
        &tty
    );

    const speed_t speed =
        baudToTermios(
            baud_
        );

    ::cfsetispeed(
        &tty,
        speed
    );

    ::cfsetospeed(
        &tty,
        speed
    );

    tty.c_cflag |=
        CLOCAL |
        CREAD;

    tty.c_cflag &=
        ~CSTOPB;

    tty.c_cflag &=
        ~CRTSCTS;

    tty.c_cflag &=
        ~PARENB;

    tty.c_cflag &=
        ~CSIZE;

    tty.c_cflag |=
        CS8;

    // Short read timing. We currently only write, but configuring
    // this makes the descriptor sane if readback is added later.
    tty.c_cc[VMIN] =
        0;

    tty.c_cc[VTIME] =
        1;

    if (
        ::tcsetattr(
            fd,
            TCSANOW,
            &tty
        ) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tcsetattr(%s) failed: %s",
            port_.c_str(),
            std::strerror(errno)
        );

        ::close(fd);

        return false;
    }

    // Switch back to blocking writes after the non-blocking open.
    const int flags =
        ::fcntl(
            fd,
            F_GETFL,
            0
        );

    if (flags >= 0)
    {
        ::fcntl(
            fd,
            F_SETFL,
            flags & ~O_NONBLOCK
        );
    }

    serial_fd_ =
        fd;

    RCLCPP_INFO(
        get_logger(),
        "Opened serial port %s at %d baud",
        port_.c_str(),
        baud_
    );

    return true;
}


void Servo2040BridgeNode::closeSerial()
{
    if (serial_fd_ < 0)
    {
        return;
    }

    ::close(
        serial_fd_
    );

    serial_fd_ =
        -1;
}


bool Servo2040BridgeNode::writeCommand(
    const std::string& payload)
{
    if (serial_fd_ < 0)
    {
        return false;
    }

    const std::string line =
        payload + "\n";

    std::size_t writtenTotal =
        0;

    while (
        writtenTotal <
        line.size())
    {
        const ssize_t written =
            ::write(
                serial_fd_,
                line.data() +
                    writtenTotal,
                line.size() -
                    writtenTotal
            );

        if (written < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }

            RCLCPP_ERROR(
                get_logger(),
                "Serial write failed: %s",
                std::strerror(errno)
            );

            return false;
        }

        if (written == 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Serial write returned 0 bytes"
            );

            return false;
        }

        writtenTotal +=
            static_cast<std::size_t>(
                written
            );
    }

    if (
        ::tcdrain(
            serial_fd_
        ) != 0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tcdrain failed after serial write: %s",
            std::strerror(errno)
        );

        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Servo2040 serial write succeeded: %s",
        payload.c_str()
    );

    return true;
}


void Servo2040BridgeNode::publishStatus(
    bool success,
    const std::string& payload)
{
    std_msgs::msg::String status;

    status.data =
        std::string(
            success
                ? "OK,"
                : "FAIL,"
        ) +
        payload;

    status_publisher_->publish(
        status
    );
}
