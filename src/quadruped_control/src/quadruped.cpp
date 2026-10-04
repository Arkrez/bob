#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "quadruped_control/Nodes/QuadrupedControllerNode.hpp"
#include "quadruped_control/Nodes/Servo2040BridgeNode.hpp"
#include "quadruped_control/Nodes/CameraDisplayNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controller =
        std::make_shared<QuadrupedControllerNode>();

    auto bridge =
        std::make_shared<Servo2040BridgeNode>();

    auto cameraDisplay =
        std::make_shared<CameraDisplayNode>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(controller);
    executor.add_node(bridge);
    executor.add_node(cameraDisplay);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}