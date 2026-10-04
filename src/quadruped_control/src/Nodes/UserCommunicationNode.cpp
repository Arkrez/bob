#include "quadruped_control/Nodes/UserCommunicationNode.hpp"

UserCommunicationNode::UserCommunicationNode()
    : Node("user_communication_node")
{
    publisher_ = create_publisher<std_msgs::msg::String>(
        "/command_received",
        10
    );




};
