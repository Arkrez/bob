#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

// A valid IK result can either hit the requested XYZ exactly or be the
// closest joint-limited pose we could find.
struct JointAngles
{
    double joint1 = 0.0;
    double joint2 = 0.0;
    double joint3 = 0.0;
    double joint4 = 0.0;

    // true means these angles are valid and may be commanded.
    bool reachable = false;

    // true only when the requested XYZ itself was solved.
    // false means this is the closest reachable fallback.
    bool exact = false;

    // FK result for these angles, body-frame inches.
    double achievedX = 0.0;
    double achievedY = 0.0;
    double achievedZ = 0.0;

    // Euclidean XYZ error from the requested target, in inches.
    double targetError = 0.0;
};

struct FootPosition
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool valid = false;
};

class QuadrupedControllerNode : public rclcpp::Node
{
public:
    QuadrupedControllerNode();

private:
    struct PendingPhysicalCommand
    {
        bool active = false;
        std::string payload;
        JointAngles angles;
    };

    // ------------------------------------------------------------
    // Publishers
    // ------------------------------------------------------------

    // Real Servo2040 bridge command.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        servo_command_publisher_;

    // Webots ros2_control JointGroupPositionController command.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        simulation_joint_publisher_;

    // Current confirmed foot position.
    // Payload: leg,x,y,z
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        leg_xyz_publisher_;

    // ------------------------------------------------------------
    // Subscriptions
    // ------------------------------------------------------------

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        ik_target_subscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        set_angles_subscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        jdirection_subscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        print_jdirection_subscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        joint_offset_subscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        print_joint_offset_subscription_;

    // Input: "0", "1", "2", "3", or "all".
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        print_leg_xyz_subscription_;

    // From Servo2040BridgeNode:
    //   OK,<original servo payload>
    //   FAIL,<original servo payload>
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        servo_command_status_subscription_;

    // ------------------------------------------------------------
    // Physical servo direction matrix
    // ------------------------------------------------------------

    double jDirection_[4][4] =
    {
        {-1.0,  1.0, -1.0, -1.0}, // 0 front left
        { 1.0, -1.0,  1.0,  1.0}, // 1 front right
        {-1.0,  1.0, -1.0, -1.0}, // 2 back left
        { 1.0,  1.0, -1.0,  1.0}  // 3 back right
    };

    // ------------------------------------------------------------
    // KDL link offsets, inches
    //
    // [leg][0] = J1 -> J3
    // [leg][1] = unused J2 slot
    // [leg][2] = J3 -> J4
    // [leg][3] = J4 -> foot
    // ------------------------------------------------------------

    double jointOffset_[4][4][3] =
    {
        {
            {0.0,  2.0,  0.0},
            {0.0,  0.0,  0.0},
            {0.0,  0.0, -2.5},
            {0.0,  0.0, -2.0}
        }, // 0 front left

        {
            {0.0, -2.0,  0.0},
            {0.0,  0.0,  0.0},
            {0.0,  0.0, -2.5},
            {0.0,  0.0, -2.0}
        }, // 1 front right

        {
            {0.0,  2.0,  0.0},
            {0.0,  0.0,  0.0},
            {0.0,  0.0, -2.5},
            {0.0,  0.0, -2.0}
        }, // 2 back left

        {
            {0.0, -2.0,  0.0},
            {0.0,  0.0,  0.0},
            {0.0,  0.0, -2.5},
            {0.0,  0.0, -2.0}
        }  // 3 back right
    };

    // ------------------------------------------------------------
    // Confirmed command state
    //
    // This is intentionally NOT updated by solveIK().
    //
    // Webots:
    //   updated after a 12-joint command is successfully published
    //   while the position controller has a subscriber.
    //
    // Physical-only:
    //   updated only after Servo2040BridgeNode reports OK for the
    //   matching serial write.
    //
    // Per leg: [J1, J3, J4] mathematical degrees.
    // ------------------------------------------------------------

    double confirmedJointAngles_[4][3] =
    {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    bool hasConfirmedJointAngles_[4] =
    {
        false,
        false,
        false,
        false
    };

    PendingPhysicalCommand pendingPhysical_[4];

    // ------------------------------------------------------------
    // IK / FK
    // ------------------------------------------------------------

    JointAngles solveIK(
        int leg,
        double x,
        double y,
        double z
    );

    FootPosition getLegXYZ(
        int leg,
        const JointAngles& angles
    ) const;

    JointAngles getConfirmedAngles(
        int leg
    ) const;

    // ------------------------------------------------------------
    // Command / state handling
    // ------------------------------------------------------------

    bool publishJointCommand(
        int leg,
        const JointAngles& angles
    );

    bool publishSimulationCommand(
        int leg,
        const JointAngles& angles
    );

    bool publishPhysicalCommand(
        int leg,
        const JointAngles& angles,
        bool waitForAckBeforeCommit
    );

    void commitJointState(
        int leg,
        const JointAngles& angles,
        const char* source
    );

    void handleServoCommandStatus(
        const std::string& status
    );

    bool validJointAngles(
        const JointAngles& angles
    ) const;

    // ------------------------------------------------------------
    // Diagnostics / printing
    // ------------------------------------------------------------

    void printLegXYZ(int leg);
    void printAllLegXYZ();
    void publishLegXYZ(int leg);

    void printJDirection();
    void printJointOffsets();
};
