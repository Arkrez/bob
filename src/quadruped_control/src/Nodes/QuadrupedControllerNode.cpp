#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <sstream>
#include <string>

#include <Eigen/Core>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "quadruped_control/Nodes/QuadrupedControllerNode.hpp"

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

constexpr double HIP_Z = 4.5;
constexpr double HIP_X = 3.0;
constexpr double HIP_Y = 3.0;

constexpr double JOINT_MIN_DEG = -90.0;
constexpr double JOINT_MAX_DEG =  90.0;

constexpr double EXACT_CARTESIAN_TOLERANCE_IN = 0.001;
constexpr double SCORE_TIE_EPSILON = 1e-12;

KDL::Vector getHipAnchor(int leg)
{
    switch (leg)
    {
        case 0:
            return KDL::Vector(HIP_X, HIP_Y, HIP_Z);      // front left
        case 1:
            return KDL::Vector(HIP_X, -HIP_Y, HIP_Z);     // front right
        case 2:
            return KDL::Vector(-HIP_X, HIP_Y, HIP_Z);     // back left
        case 3:
            return KDL::Vector(-HIP_X, -HIP_Y, HIP_Z);    // back right
        default:
            return KDL::Vector(0.0, 0.0, HIP_Z);
    }
}

KDL::Chain getLegChain(
    int leg,
    const double offsets[4][4][3])
{
    KDL::Chain chain;

    // Body frame -> this leg's body-corner J1 pivot.
    chain.addSegment(
        KDL::Segment(
            "body_to_hip",
            KDL::Joint(
                "body_to_hip_fixed",
                KDL::Joint::None
            ),
            KDL::Frame(
                getHipAnchor(leg)
            )
        )
    );

    // J1: hip yaw.
    chain.addSegment(
        KDL::Segment(
            "link1",
            KDL::Joint(
                "joint1",
                KDL::Joint::RotZ
            ),
            KDL::Frame(
                KDL::Vector(
                    offsets[leg][0][0],
                    offsets[leg][0][1],
                    offsets[leg][0][2]
                )
            )
        )
    );

    // J3: shoulder pitch.
    // The current Webots/URDF model uses the X axis for J3/J4.
    chain.addSegment(
        KDL::Segment(
            "link2",
            KDL::Joint(
                "joint3",
                KDL::Joint::RotX
            ),
            KDL::Frame(
                KDL::Vector(
                    offsets[leg][2][0],
                    offsets[leg][2][1],
                    offsets[leg][2][2]
                )
            )
        )
    );

    // J4: knee pitch, relative to J3.
    chain.addSegment(
        KDL::Segment(
            "link3",
            KDL::Joint(
                "joint4",
                KDL::Joint::RotX
            ),
            KDL::Frame(
                KDL::Vector(
                    offsets[leg][3][0],
                    offsets[leg][3][1],
                    offsets[leg][3][2]
                )
            )
        )
    );

    return chain;
}

bool isFinite(double value)
{
    return std::isfinite(value);
}

bool jointInRange(double deg)
{
    return isFinite(deg) &&
           deg >= JOINT_MIN_DEG &&
           deg <= JOINT_MAX_DEG;
}

double toServoDegJ1(double jointDeg)
{
    return 90.0 + jointDeg;
}

double toServoDegJ3(double jointDeg)
{
    return 90.0 + jointDeg;
}

double toServoDegJ4(double jointDeg)
{
    return 90.0 + jointDeg;
}

double applyDirection(
    double servoDeg,
    double direction)
{
    return 90.0 +
           direction *
           (servoDeg - 90.0);
}

double clampServo(
    double servoDeg,
    const char* label,
    rclcpp::Logger logger)
{
    constexpr double EPSILON = 0.001;

    if (servoDeg < 0.0 && servoDeg > -EPSILON)
    {
        servoDeg = 0.0;
    }

    if (servoDeg > 180.0 && servoDeg < 180.0 + EPSILON)
    {
        servoDeg = 180.0;
    }

    if (servoDeg < 0.0 || servoDeg > 180.0)
    {
        RCLCPP_WARN(
            logger,
            "%s servo angle %.2f out of [0,180], clamping",
            label,
            servoDeg
        );
    }

    return std::max(
        0.0,
        std::min(
            180.0,
            servoDeg
        )
    );
}

bool computeFK(
    int leg,
    double j1Deg,
    double j3Deg,
    double j4Deg,
    const double offsets[4][4][3],
    KDL::Vector& position)
{
    KDL::Chain chain =
        getLegChain(
            leg,
            offsets
        );

    KDL::JntArray joints(3);

    joints(0) = j1Deg * DEG_TO_RAD;
    joints(1) = j3Deg * DEG_TO_RAD;
    joints(2) = j4Deg * DEG_TO_RAD;

    KDL::ChainFkSolverPos_recursive solver(chain);

    KDL::Frame result;

    if (solver.JntToCart(joints, result) < 0)
    {
        return false;
    }

    position = result.p;

    return
        isFinite(position.x()) &&
        isFinite(position.y()) &&
        isFinite(position.z());
}

double squared(
    double value)
{
    return value * value;
}

double jointDistanceSquared(
    double j1,
    double j3,
    double j4,
    double currentJ1,
    double currentJ3,
    double currentJ4)
{
    return
        squared(j1 - currentJ1) +
        squared(j3 - currentJ3) +
        squared(j4 - currentJ4);
}

std::string lowerCopy(std::string text)
{
    std::transform(
        text.begin(),
        text.end(),
        text.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        }
    );

    return text;
}

} // namespace


// ============================================================
// CONSTRUCTOR
// ============================================================

QuadrupedControllerNode::QuadrupedControllerNode()
    : Node("quadruped_controller")
{
    servo_command_publisher_ =
        create_publisher<std_msgs::msg::String>(
            "/servo_commands",
            10
        );

    simulation_joint_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
            "/leg_position_controller/commands",
            10
        );

    leg_xyz_publisher_ =
        create_publisher<std_msgs::msg::String>(
            "/leg_xyz",
            10
        );


    // ========================================================
    // /set_angles
    //
    // Format:
    //   leg,j1,j3,j4
    //
    // Angles are mathematical degrees.
    // ========================================================

    set_angles_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/set_angles",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                int leg;
                double j1;
                double j3;
                double j4;

                const int parsed =
                    std::sscanf(
                        msg->data.c_str(),
                        "%d,%lf,%lf,%lf",
                        &leg,
                        &j1,
                        &j3,
                        &j4
                    );

                if (parsed != 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad /set_angles message: '%s'",
                        msg->data.c_str()
                    );
                    return;
                }

                if (leg < 0 || leg >= 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad leg index: %d",
                        leg
                    );
                    return;
                }

                JointAngles angles;

                angles.joint1 = j1;
                angles.joint2 = 0.0;
                angles.joint3 = j3;
                angles.joint4 = j4;
                angles.reachable = true;
                angles.exact = true;

                FootPosition proposed =
                    getLegXYZ(
                        leg,
                        angles
                    );

                if (proposed.valid)
                {
                    angles.achievedX = proposed.x;
                    angles.achievedY = proposed.y;
                    angles.achievedZ = proposed.z;
                }

                if (!publishJointCommand(leg, angles))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Leg %d set_angles command was not accepted by "
                        "Webots and was not queued for confirmed physical write",
                        leg
                    );
                }
            }
        );


    // ========================================================
    // /ik_target
    //
    // Format:
    //   leg,x,y,z
    //
    // Coordinates are body-frame inches.
    // ========================================================

    ik_target_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/ik_target",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                int leg;
                double x;
                double y;
                double z;

                const int parsed =
                    std::sscanf(
                        msg->data.c_str(),
                        "%d,%lf,%lf,%lf",
                        &leg,
                        &x,
                        &y,
                        &z
                    );

                if (parsed != 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad /ik_target message: '%s'",
                        msg->data.c_str()
                    );
                    return;
                }

                if (leg < 0 || leg >= 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad leg index: %d",
                        leg
                    );
                    return;
                }

                if (!isFinite(x) || !isFinite(y) || !isFinite(z))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Leg %d received non-finite IK target",
                        leg
                    );
                    return;
                }

                RCLCPP_INFO(
                    get_logger(),
                    "Leg %d target body=(%.3f, %.3f, %.3f)",
                    leg,
                    x,
                    y,
                    z
                );

                JointAngles angles =
                    solveIK(
                        leg,
                        x,
                        y,
                        z
                    );

                if (!angles.reachable)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "No valid joint configuration could be produced for "
                        "leg %d. State was NOT changed.",
                        leg
                    );
                    return;
                }

                if (angles.exact)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "Leg %d exact IK -> "
                        "J1=%.3f J3=%.3f J4=%.3f "
                        "XYZ=(%.3f, %.3f, %.3f)",
                        leg,
                        angles.joint1,
                        angles.joint3,
                        angles.joint4,
                        angles.achievedX,
                        angles.achievedY,
                        angles.achievedZ
                    );
                }
                else
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Leg %d target is unreachable. "
                        "Using closest reachable XYZ=(%.3f, %.3f, %.3f), "
                        "error=%.3f in, angles=(%.3f, %.3f, %.3f)",
                        leg,
                        angles.achievedX,
                        angles.achievedY,
                        angles.achievedZ,
                        angles.targetError,
                        angles.joint1,
                        angles.joint3,
                        angles.joint4
                    );
                }

                if (!publishJointCommand(leg, angles))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Leg %d IK produced valid angles, but no output path "
                        "accepted the command. State was NOT changed.",
                        leg
                    );
                }
            }
        );


    // ========================================================
    // /jdirection
    //
    // Format:
    //   leg,joint,value
    // ========================================================

    jdirection_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/jdirection",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                int leg;
                int joint;
                double value;

                const int parsed =
                    std::sscanf(
                        msg->data.c_str(),
                        "%d,%d,%lf",
                        &leg,
                        &joint,
                        &value
                    );

                if (parsed != 3)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Expected /jdirection: leg,joint,value"
                    );
                    return;
                }

                if (
                    leg < 0 ||
                    leg >= 4 ||
                    joint < 0 ||
                    joint >= 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad jDirection index"
                    );
                    return;
                }

                if (value != 1.0 && value != -1.0)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "jDirection should normally be +1 or -1. "
                        "Received %.2f",
                        value
                    );
                }

                const double oldValue =
                    jDirection_[leg][joint];

                jDirection_[leg][joint] =
                    value;

                RCLCPP_INFO(
                    get_logger(),
                    "Changed jDirection[%d][%d]: %.2f -> %.2f",
                    leg,
                    joint,
                    oldValue,
                    value
                );

                printJDirection();
            }
        );


    // ========================================================
    // /print_jdirection
    // ========================================================

    print_jdirection_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/print_jdirection",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr)
            {
                printJDirection();
            }
        );


    // ========================================================
    // /joint_offset
    //
    // Format:
    //   leg,joint,x,y,z
    // ========================================================

    joint_offset_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/joint_offset",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                int leg;
                int joint;

                double x;
                double y;
                double z;

                const int parsed =
                    std::sscanf(
                        msg->data.c_str(),
                        "%d,%d,%lf,%lf,%lf",
                        &leg,
                        &joint,
                        &x,
                        &y,
                        &z
                    );

                if (parsed != 5)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad /joint_offset message: '%s'",
                        msg->data.c_str()
                    );

                    RCLCPP_WARN(
                        get_logger(),
                        "Expected: leg,joint,x,y,z"
                    );

                    return;
                }

                if (
                    leg < 0 ||
                    leg >= 4 ||
                    joint < 0 ||
                    joint >= 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Bad joint offset index"
                    );
                    return;
                }

                if (!isFinite(x) || !isFinite(y) || !isFinite(z))
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Joint offset must contain finite numbers"
                    );
                    return;
                }

                jointOffset_[leg][joint][0] = x;
                jointOffset_[leg][joint][1] = y;
                jointOffset_[leg][joint][2] = z;

                RCLCPP_INFO(
                    get_logger(),
                    "Changed jointOffset[%d][%d] -> (%.3f, %.3f, %.3f)",
                    leg,
                    joint,
                    x,
                    y,
                    z
                );

                printJointOffsets();

                if (hasConfirmedJointAngles_[leg])
                {
                    publishLegXYZ(leg);
                }
            }
        );


    // ========================================================
    // /print_joint_offsets
    // ========================================================

    print_joint_offset_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/print_joint_offsets",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr)
            {
                printJointOffsets();
            }
        );


    // ========================================================
    // /print_leg_xyz
    //
    // data:
    //   "0", "1", "2", "3", or "all"
    // ========================================================

    print_leg_xyz_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/print_leg_xyz",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                const std::string request =
                    lowerCopy(msg->data);

                if (request == "all")
                {
                    printAllLegXYZ();
                    return;
                }

                int leg = -1;

                if (
                    std::sscanf(
                        request.c_str(),
                        "%d",
                        &leg
                    ) != 1 ||
                    leg < 0 ||
                    leg >= 4)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Expected /print_leg_xyz data to be "
                        "'0', '1', '2', '3', or 'all'"
                    );
                    return;
                }

                printLegXYZ(leg);
            }
        );


    // ========================================================
    // /servo_command_status
    //
    // From Servo2040BridgeNode:
    //   OK,<original payload>
    //   FAIL,<original payload>
    // ========================================================

    servo_command_status_subscription_ =
        create_subscription<std_msgs::msg::String>(
            "/servo_command_status",
            10,
            [this](
                const std_msgs::msg::String::SharedPtr msg)
            {
                handleServoCommandStatus(
                    msg->data
                );
            }
        );


    RCLCPP_INFO(
        get_logger(),
        "Quadruped controller started."
    );

    RCLCPP_INFO(
        get_logger(),
        "Leg numbering: "
        "0=front-left 1=front-right 2=back-left 3=back-right"
    );

    RCLCPP_INFO(
        get_logger(),
        "XYZ frame: body coordinates, inches, "
        "+X forward +Y left +Z up"
    );

    printJDirection();
    printJointOffsets();
}


// ============================================================
// VALIDATE JOINT COMMAND
// ============================================================

bool QuadrupedControllerNode::validJointAngles(
    const JointAngles& angles) const
{
    return
        jointInRange(angles.joint1) &&
        jointInRange(angles.joint3) &&
        jointInRange(angles.joint4);
}


// ============================================================
// PUBLISH ONE JOINT COMMAND
//
// State rules:
//
// 1. If Webots has a subscriber and the 12-joint command is
//    published, commit immediately.
//
// 2. If there is no Webots subscriber but the physical bridge
//    exists, queue the command and wait for /servo_command_status.
//
// 3. If no output path accepts the command, do not update state.
// ============================================================
bool QuadrupedControllerNode::publishJointCommand(
    int leg,
    const JointAngles& angles)
{
    if (leg < 0 || leg >= 4)
    {
        return false;
    }

    if (!angles.reachable || !validJointAngles(angles))
    {
        RCLCPP_WARN(
            get_logger(),
            "Refusing invalid joint command for leg %d: "
            "(%.3f, %.3f, %.3f)",
            leg,
            angles.joint1,
            angles.joint3,
            angles.joint4
        );

        return false;
    }

    const bool simulationPublished =
        publishSimulationCommand(
            leg,
            angles
        );

    const bool physicalQueued =
        publishPhysicalCommand(
            leg,
            angles,
            !simulationPublished
        );

    if (simulationPublished)
    {
        commitJointState(
            leg,
            angles,
            "webots publish"
        );

        return true;
    }

    if (physicalQueued)
    {
        RCLCPP_INFO(
            get_logger(),
            "Leg %d physical command queued; waiting for serial "
            "write acknowledgement before state update",
            leg
        );

        return true;
    }

    return false;
}   // <-- THIS closing brace is probably missing


// ============================================================
// WEBOTS PUBLISH
// ============================================================
bool QuadrupedControllerNode::publishSimulationCommand(
    int targetLeg,
    const JointAngles& targetAngles)
{
    if (!simulation_joint_publisher_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "simulation_joint_publisher_ is NULL"
        );

        return false;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(12, 0.0);

    for (int leg = 0; leg < 4; ++leg)
    {
        const int index = leg * 3;

        if (leg == targetLeg)
        {
            msg.data[index + 0] =
                targetAngles.joint1 * DEG_TO_RAD;

            msg.data[index + 1] =
                targetAngles.joint3 * DEG_TO_RAD;

            msg.data[index + 2] =
                targetAngles.joint4 * DEG_TO_RAD;

            continue;
        }

        if (hasConfirmedJointAngles_[leg])
        {
            msg.data[index + 0] =
                confirmedJointAngles_[leg][0] * DEG_TO_RAD;

            msg.data[index + 1] =
                confirmedJointAngles_[leg][1] * DEG_TO_RAD;

            msg.data[index + 2] =
                confirmedJointAngles_[leg][2] * DEG_TO_RAD;
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "About to publish Webots joint command for leg %d",
        targetLeg
    );

    simulation_joint_publisher_->publish(msg);

    RCLCPP_INFO(
        get_logger(),
        "Webots joint command published for leg %d",
        targetLeg
    );

    return true;
}

// ============================================================
// PHYSICAL SERVO PUBLISH
// ============================================================

bool QuadrupedControllerNode::publishPhysicalCommand(
    int leg,
    const JointAngles& angles,
    bool waitForAckBeforeCommit)
{
    if (servo_command_publisher_->get_subscription_count() == 0)
    {
        return false;
    }

    const double servo1 =
        clampServo(
            applyDirection(
                toServoDegJ1(angles.joint1),
                jDirection_[leg][0]
            ),
            "J1",
            get_logger()
        );

    const double servo3 =
        clampServo(
            applyDirection(
                toServoDegJ3(angles.joint3),
                jDirection_[leg][2]
            ),
            "J3",
            get_logger()
        );

    const double servo4 =
        clampServo(
            applyDirection(
                toServoDegJ4(angles.joint4),
                jDirection_[leg][3]
            ),
            "J4",
            get_logger()
        );

    std_msgs::msg::String out;

    out.data =
        std::to_string(leg) + "," +
        std::to_string(servo1) + "," +
        std::to_string(servo3) + "," +
        std::to_string(servo4);

    if (waitForAckBeforeCommit)
    {
        // Store BEFORE publishing. A single-threaded executor will run
        // the bridge status callback after this callback returns, and a
        // multi-threaded executor is also safe from missing the expected
        // payload due to this ordering.
        pendingPhysical_[leg].active = true;
        pendingPhysical_[leg].payload = out.data;
        pendingPhysical_[leg].angles = angles;
    }
    else
    {
        pendingPhysical_[leg].active = false;
        pendingPhysical_[leg].payload.clear();
    }

    try
    {
        servo_command_publisher_->publish(out);
    }
    catch (const std::exception& error)
    {
        if (waitForAckBeforeCommit)
        {
            pendingPhysical_[leg].active = false;
            pendingPhysical_[leg].payload.clear();
        }

        RCLCPP_ERROR(
            get_logger(),
            "Servo command ROS publish failed: %s",
            error.what()
        );

        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Servo command published for leg %d: %s",
        leg,
        out.data.c_str()
    );

    return true;
}


// ============================================================
// SERIAL BRIDGE STATUS
// ============================================================

void QuadrupedControllerNode::handleServoCommandStatus(
    const std::string& status)
{
    bool success = false;
    std::string payload;

    if (status.rfind("OK,", 0) == 0)
    {
        success = true;
        payload = status.substr(3);
    }
    else if (status.rfind("FAIL,", 0) == 0)
    {
        success = false;
        payload = status.substr(5);
    }
    else
    {
        RCLCPP_WARN(
            get_logger(),
            "Unknown /servo_command_status: '%s'",
            status.c_str()
        );

        return;
    }

    int leg = -1;

    if (
        std::sscanf(
            payload.c_str(),
            "%d,",
            &leg
        ) != 1 ||
        leg < 0 ||
        leg >= 4)
    {
        RCLCPP_WARN(
            get_logger(),
            "Could not parse leg from servo status payload '%s'",
            payload.c_str()
        );

        return;
    }

    // Ignore ACKs for old commands. This prevents a delayed response
    // from rolling state backward after a newer command was issued.
    if (
        !pendingPhysical_[leg].active ||
        pendingPhysical_[leg].payload != payload)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "Ignoring stale/untracked servo status for leg %d",
            leg
        );

        return;
    }

    if (success)
    {
        const JointAngles confirmed =
            pendingPhysical_[leg].angles;

        pendingPhysical_[leg].active = false;
        pendingPhysical_[leg].payload.clear();

        commitJointState(
            leg,
            confirmed,
            "successful serial write"
        );
    }
    else
    {
        pendingPhysical_[leg].active = false;
        pendingPhysical_[leg].payload.clear();

        RCLCPP_WARN(
            get_logger(),
            "Servo serial write failed for leg %d. "
            "Confirmed state was NOT changed.",
            leg
        );
    }
}


// ============================================================
// COMMIT CONFIRMED STATE
// ============================================================

void QuadrupedControllerNode::commitJointState(
    int leg,
    const JointAngles& angles,
    const char* source)
{
    confirmedJointAngles_[leg][0] =
        angles.joint1;

    confirmedJointAngles_[leg][1] =
        angles.joint3;

    confirmedJointAngles_[leg][2] =
        angles.joint4;

    hasConfirmedJointAngles_[leg] =
        true;

    const FootPosition position =
        getLegXYZ(
            leg,
            angles
        );

    if (position.valid)
    {
        RCLCPP_INFO(
            get_logger(),
            "Confirmed leg %d state via %s: "
            "J=(%.3f, %.3f, %.3f) "
            "XYZ=(%.3f, %.3f, %.3f) in",
            leg,
            source,
            angles.joint1,
            angles.joint3,
            angles.joint4,
            position.x,
            position.y,
            position.z
        );

        publishLegXYZ(leg);
    }
    else
    {
        RCLCPP_WARN(
            get_logger(),
            "Leg %d joint state was confirmed, but FK failed",
            leg
        );
    }
}


// ============================================================
// CURRENT ANGLES
// ============================================================

JointAngles QuadrupedControllerNode::getConfirmedAngles(
    int leg) const
{
    JointAngles angles;

    if (
        leg < 0 ||
        leg >= 4 ||
        !hasConfirmedJointAngles_[leg])
    {
        return angles;
    }

    angles.joint1 =
        confirmedJointAngles_[leg][0];

    angles.joint2 =
        0.0;

    angles.joint3 =
        confirmedJointAngles_[leg][1];

    angles.joint4 =
        confirmedJointAngles_[leg][2];

    angles.reachable =
        true;

    angles.exact =
        true;

    return angles;
}


// ============================================================
// FK / XYZ
// ============================================================

FootPosition QuadrupedControllerNode::getLegXYZ(
    int leg,
    const JointAngles& angles) const
{
    FootPosition position;

    if (
        leg < 0 ||
        leg >= 4 ||
        !validJointAngles(angles))
    {
        return position;
    }

    KDL::Vector result;

    if (
        !computeFK(
            leg,
            angles.joint1,
            angles.joint3,
            angles.joint4,
            jointOffset_,
            result
        ))
    {
        return position;
    }

    position.x = result.x();
    position.y = result.y();
    position.z = result.z();
    position.valid = true;

    return position;
}


void QuadrupedControllerNode::publishLegXYZ(
    int leg)
{
    if (
        leg < 0 ||
        leg >= 4 ||
        !hasConfirmedJointAngles_[leg])
    {
        return;
    }

    const JointAngles angles =
        getConfirmedAngles(leg);

    const FootPosition position =
        getLegXYZ(
            leg,
            angles
        );

    if (!position.valid)
    {
        return;
    }

    std::ostringstream stream;

    stream.setf(std::ios::fixed);
    stream.precision(6);

    stream
        << leg
        << ","
        << position.x
        << ","
        << position.y
        << ","
        << position.z;

    std_msgs::msg::String msg;

    msg.data =
        stream.str();

    leg_xyz_publisher_->publish(msg);
}


void QuadrupedControllerNode::printLegXYZ(
    int leg)
{
    if (
        leg < 0 ||
        leg >= 4)
    {
        RCLCPP_WARN(
            get_logger(),
            "Bad leg index: %d",
            leg
        );

        return;
    }

    if (!hasConfirmedJointAngles_[leg])
    {
        RCLCPP_WARN(
            get_logger(),
            "Leg %d has no confirmed joint state yet",
            leg
        );

        return;
    }

    const JointAngles angles =
        getConfirmedAngles(leg);

    const FootPosition position =
        getLegXYZ(
            leg,
            angles
        );

    if (!position.valid)
    {
        RCLCPP_WARN(
            get_logger(),
            "FK failed for leg %d",
            leg
        );

        return;
    }

    static constexpr const char* LEG_NAMES[4] =
    {
        "FL",
        "FR",
        "BL",
        "BR"
    };

    RCLCPP_INFO(
        get_logger(),
        "Leg %d %s XYZ=(%.6f, %.6f, %.6f) inches "
        "J=(%.3f, %.3f, %.3f) deg",
        leg,
        LEG_NAMES[leg],
        position.x,
        position.y,
        position.z,
        angles.joint1,
        angles.joint3,
        angles.joint4
    );

    publishLegXYZ(leg);
}


void QuadrupedControllerNode::printAllLegXYZ()
{
    for (int leg = 0; leg < 4; ++leg)
    {
        printLegXYZ(leg);
    }
}


// ============================================================
// PRINT JDIRECTION
// ============================================================

void QuadrupedControllerNode::printJDirection()
{
    RCLCPP_INFO(
        get_logger(),
        "\n"
        "================ jDirection ================\n"
        "             J1      J2      J3      J4\n"
        "Leg 0 FL:  %5.1f   %5.1f   %5.1f   %5.1f\n"
        "Leg 1 FR:  %5.1f   %5.1f   %5.1f   %5.1f\n"
        "Leg 2 BL:  %5.1f   %5.1f   %5.1f   %5.1f\n"
        "Leg 3 BR:  %5.1f   %5.1f   %5.1f   %5.1f\n"
        "============================================",
        jDirection_[0][0],
        jDirection_[0][1],
        jDirection_[0][2],
        jDirection_[0][3],

        jDirection_[1][0],
        jDirection_[1][1],
        jDirection_[1][2],
        jDirection_[1][3],

        jDirection_[2][0],
        jDirection_[2][1],
        jDirection_[2][2],
        jDirection_[2][3],

        jDirection_[3][0],
        jDirection_[3][1],
        jDirection_[3][2],
        jDirection_[3][3]
    );
}


// ============================================================
// PRINT JOINT OFFSETS
// ============================================================

void QuadrupedControllerNode::printJointOffsets()
{
    static constexpr const char* LEG_NAMES[4] =
    {
        "FL",
        "FR",
        "BL",
        "BR"
    };

    for (int leg = 0; leg < 4; ++leg)
    {
        RCLCPP_INFO(
            get_logger(),
            "Leg %d %s offsets: "
            "J1=(%.2f,%.2f,%.2f) "
            "J2=(%.2f,%.2f,%.2f) "
            "J3=(%.2f,%.2f,%.2f) "
            "J4=(%.2f,%.2f,%.2f)",
            leg,
            LEG_NAMES[leg],

            jointOffset_[leg][0][0],
            jointOffset_[leg][0][1],
            jointOffset_[leg][0][2],

            jointOffset_[leg][1][0],
            jointOffset_[leg][1][1],
            jointOffset_[leg][1][2],

            jointOffset_[leg][2][0],
            jointOffset_[leg][2][1],
            jointOffset_[leg][2][2],

            jointOffset_[leg][3][0],
            jointOffset_[leg][3][1],
            jointOffset_[leg][3][2]
        );
    }
}


// ============================================================
// IK
//
// Behavior:
//   1. Try exact multi-seed KDL LMA IK.
//   2. Enforce +/-90 degree joint limits.
//   3. If exact target is impossible, search the valid joint space
//      and return the FK point with minimum Euclidean XYZ error.
//   4. NEVER mutate confirmed state here.
// ============================================================

JointAngles QuadrupedControllerNode::solveIK(
    int leg,
    double x,
    double y,
    double z)
{
    JointAngles result;

    if (
        leg < 0 ||
        leg >= 4 ||
        !finite(x) ||
        !finite(y) ||
        !finite(z))
    {
        return result;
    }

    const KDL::Vector requested(
        x,
        y,
        z
    );

    KDL::Chain chain =
        getLegChain(
            leg,
            jointOffset_
        );

    KDL::ChainFkSolverPos_recursive fkSolver(
        chain
    );

    double currentJ1 = 0.0;
    double currentJ3 = 0.0;
    double currentJ4 = 0.0;

    if (hasConfirmedJointAngles_[leg])
    {
        currentJ1 =
            confirmedJointAngles_[leg][0];

        currentJ3 =
            confirmedJointAngles_[leg][1];

        currentJ4 =
            confirmedJointAngles_[leg][2];
    }


    // --------------------------------------------------------
    // 1) Exact KDL LMA search
    // --------------------------------------------------------

    Eigen::Matrix<double, 6, 1> weights;

    weights <<
        1.0,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0;

    KDL::ChainIkSolverPos_LMA solver(
        chain,
        weights,
        1e-5,
        200
    );

    KDL::Frame target(
        KDL::Rotation::Identity(),
        requested
    );

    const double seedOffsets[] =
    {
         0.0,
        10.0,
       -10.0,
        25.0,
       -25.0,
        45.0,
       -45.0,
        70.0,
       -70.0
    };

    bool exactFound = false;
    double exactBestJointDistance =
        std::numeric_limits<double>::infinity();

    KDL::JntArray exactBest(3);
    KDL::Vector exactBestPosition;

    for (double j1Offset : seedOffsets)
    {
        for (double j3Offset : seedOffsets)
        {
            for (double j4Offset : seedOffsets)
            {
                KDL::JntArray initial(3);

                initial(0) =
                    (currentJ1 + j1Offset) *
                    DEG_TO_RAD;

                initial(1) =
                    (currentJ3 + j3Offset) *
                    DEG_TO_RAD;

                initial(2) =
                    (currentJ4 + j4Offset) *
                    DEG_TO_RAD;

                KDL::JntArray candidate(3);

                const int ikResult =
                    solver.CartToJnt(
                        initial,
                        target,
                        candidate
                    );

                if (ikResult < 0)
                {
                    continue;
                }

                const double j1 =
                    candidate(0) *
                    RAD_TO_DEG;

                const double j3 =
                    candidate(1) *
                    RAD_TO_DEG;

                const double j4 =
                    candidate(2) *
                    RAD_TO_DEG;

                if (
                    !jointInRange(j1) ||
                    !jointInRange(j3) ||
                    !jointInRange(j4))
                {
                    continue;
                }

                KDL::Frame actual;

                if (
                    fkSolver.JntToCart(
                        candidate,
                        actual
                    ) < 0)
                {
                    continue;
                }

                const KDL::Vector error =
                    actual.p -
                    requested;

                const double cartesianError =
                    error.Norm();

                if (
                    !finite(cartesianError) ||
                    cartesianError >
                        EXACT_CARTESIAN_TOLERANCE_IN)
                {
                    continue;
                }

                const double moveDistance =
                    jointDistanceSquared(
                        j1,
                        j3,
                        j4,
                        currentJ1,
                        currentJ3,
                        currentJ4
                    );

                if (
                    !exactFound ||
                    moveDistance <
                        exactBestJointDistance)
                {
                    exactFound = true;

                    exactBestJointDistance =
                        moveDistance;

                    exactBest(0) =
                        candidate(0);

                    exactBest(1) =
                        candidate(1);

                    exactBest(2) =
                        candidate(2);

                    exactBestPosition =
                        actual.p;
                }
            }
        }
    }

    if (exactFound)
    {
        result.joint1 =
            exactBest(0) *
            RAD_TO_DEG;

        result.joint2 =
            0.0;

        result.joint3 =
            exactBest(1) *
            RAD_TO_DEG;

        result.joint4 =
            exactBest(2) *
            RAD_TO_DEG;

        result.reachable =
            true;

        result.exact =
            true;

        result.achievedX =
            exactBestPosition.x();

        result.achievedY =
            exactBestPosition.y();

        result.achievedZ =
            exactBestPosition.z();

        result.targetError =
            (
                exactBestPosition -
                requested
            ).Norm();

        return result;
    }


    // --------------------------------------------------------
    // 2) Closest reachable fallback
    //
    // This searches only legal joint angles. It minimizes
    // Cartesian XYZ error, not joint error.
    //
    // Joint distance is used only as a tie breaker so the robot
    // does not arbitrarily jump to a different IK branch.
    // --------------------------------------------------------

    struct BestCandidate
    {
        bool found = false;

        double j1 = 0.0;
        double j3 = 0.0;
        double j4 = 0.0;

        double cartesianErrorSquared =
            std::numeric_limits<double>::infinity();

        double jointMoveSquared =
            std::numeric_limits<double>::infinity();

        KDL::Vector position;
    };

    BestCandidate best;

    auto evaluate =
        [&](double j1, double j3, double j4)
        {
            if (
                !jointInRange(j1) ||
                !jointInRange(j3) ||
                !jointInRange(j4))
            {
                return;
            }

            KDL::JntArray joints(3);

            joints(0) =
                j1 * DEG_TO_RAD;

            joints(1) =
                j3 * DEG_TO_RAD;

            joints(2) =
                j4 * DEG_TO_RAD;

            KDL::Frame actual;

            if (
                fkSolver.JntToCart(
                    joints,
                    actual
                ) < 0)
            {
                return;
            }

            const KDL::Vector error =
                actual.p -
                requested;

            const double errorSquared =
                squared(error.x()) +
                squared(error.y()) +
                squared(error.z());

            if (!isFinite(errorSquared))
            {
                return;
            }

            const double moveSquared =
                jointDistanceSquared(
                    j1,
                    j3,
                    j4,
                    currentJ1,
                    currentJ3,
                    currentJ4
                );

            const bool betterCartesian =
                !best.found ||
                errorSquared <
                    best.cartesianErrorSquared -
                    SCORE_TIE_EPSILON;

            const bool sameCartesianButCloserJoints =
                best.found &&
                std::abs(
                    errorSquared -
                    best.cartesianErrorSquared
                ) <= SCORE_TIE_EPSILON &&
                moveSquared <
                    best.jointMoveSquared;

            if (
                betterCartesian ||
                sameCartesianButCloserJoints)
            {
                best.found =
                    true;

                best.j1 =
                    j1;

                best.j3 =
                    j3;

                best.j4 =
                    j4;

                best.cartesianErrorSquared =
                    errorSquared;

                best.jointMoveSquared =
                    moveSquared;

                best.position =
                    actual.p;
            }
        };


    // Coarse global search: the important part that prevents a
    // local IK failure from making us give up.
    for (
        double j1 = JOINT_MIN_DEG;
        j1 <= JOINT_MAX_DEG + 1e-9;
        j1 += 10.0)
    {
        for (
            double j3 = JOINT_MIN_DEG;
            j3 <= JOINT_MAX_DEG + 1e-9;
            j3 += 10.0)
        {
            for (
                double j4 = JOINT_MIN_DEG;
                j4 <= JOINT_MAX_DEG + 1e-9;
                j4 += 10.0)
            {
                evaluate(
                    j1,
                    j3,
                    j4
                );
            }
        }
    }

    if (!best.found)
    {
        return result;
    }


    // Refine around the best global-grid result.
    auto refine =
        [&](double radius, double step)
        {
            const double centerJ1 =
                best.j1;

            const double centerJ3 =
                best.j3;

            const double centerJ4 =
                best.j4;

            const double minJ1 =
                std::max(
                    JOINT_MIN_DEG,
                    centerJ1 - radius
                );

            const double maxJ1 =
                std::min(
                    JOINT_MAX_DEG,
                    centerJ1 + radius
                );

            const double minJ3 =
                std::max(
                    JOINT_MIN_DEG,
                    centerJ3 - radius
                );

            const double maxJ3 =
                std::min(
                    JOINT_MAX_DEG,
                    centerJ3 + radius
                );

            const double minJ4 =
                std::max(
                    JOINT_MIN_DEG,
                    centerJ4 - radius
                );

            const double maxJ4 =
                std::min(
                    JOINT_MAX_DEG,
                    centerJ4 + radius
                );

            for (
                double j1 = minJ1;
                j1 <= maxJ1 + 1e-9;
                j1 += step)
            {
                for (
                    double j3 = minJ3;
                    j3 <= maxJ3 + 1e-9;
                    j3 += step)
                {
                    for (
                        double j4 = minJ4;
                        j4 <= maxJ4 + 1e-9;
                        j4 += step)
                    {
                        evaluate(
                            j1,
                            j3,
                            j4
                        );
                    }
                }
            }
        };

    refine(10.0, 2.0);
    refine( 2.0, 0.5);
    refine( 0.5, 0.1);
    refine( 0.1, 0.02);

    if (!best.found)
    {
        return result;
    }

    result.joint1 =
        best.j1;

    result.joint2 =
        0.0;

    result.joint3 =
        best.j3;

    result.joint4 =
        best.j4;

    result.reachable =
        true;

    result.achievedX =
        best.position.x();

    result.achievedY =
        best.position.y();

    result.achievedZ =
        best.position.z();

    result.targetError =
        std::sqrt(
            best.cartesianErrorSquared
        );

    result.exact =
        result.targetError <=
            EXACT_CARTESIAN_TOLERANCE_IN;

    return result;
}
