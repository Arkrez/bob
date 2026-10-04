#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <array>

// ============================================================
// Helpers
// ============================================================

std::string trim(const std::string& input)
{
    const auto first = input.find_first_not_of(" \t\r\n");

    if (first == std::string::npos)
        return "";

    const auto last = input.find_last_not_of(" \t\r\n");

    return input.substr(first, last - first + 1);
}

std::string captureCommand(const std::string& command)
{
    std::array<char, 512> buffer{};
    std::string result;

    FILE* pipe = popen(command.c_str(), "r");

    if (!pipe)
        return "";

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
        result += buffer.data();

    pclose(pipe);

    return trim(result);
}

std::string shellQuote(const std::string& text)
{
    std::string result = "'";

    for (char c : text)
    {
        if (c == '\'')
            result += "'\\''";
        else
            result += c;
    }

    result += "'";

    return result;
}

// ============================================================
// Open command in a new macOS Terminal
// ============================================================

void runMacTerminalCommand(const std::string& command)
{
    std::string escaped;

    for (char c : command)
    {
        if (c == '\\')
            escaped += "\\\\";
        else if (c == '"')
            escaped += "\\\"";
        else
            escaped += c;
    }

    std::string appleScript =
        "tell application \"Terminal\"\n"
        "activate\n"
        "do script \"" + escaped + "\"\n"
        "end tell";

    std::string fullCommand =
        "osascript -e " + shellQuote(appleScript);

    std::system(fullCommand.c_str());
}

// ============================================================
// Main
// ============================================================

int main()
{
    // ========================================================
    // CONFIG
    // ========================================================

    const std::string containerName = "quadruped-humble-sim";

    const std::string rosDistro =
        "humble";

    const std::string workspace =
        "/workspace";

    const std::string packageName =
        "quadruped_control";

    const std::string executableName =
        "quadruped";

    const std::string partition =
        "quadruped_sim";

    std::cout
        << "\n========================================\n"
        << " Quadruped Humble + Gazebo Harmonic\n"
        << "========================================\n\n";

    // ========================================================
    // 1. CLEAN OLD MAC GAZEBO PROCESSES
    // ========================================================

    std::cout << "[1] Cleaning old Gazebo processes...\n";

    std::system(
        "pkill -TERM -f '[g]z sim' "
        ">/dev/null 2>&1 || true"
    );

    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );

    // ========================================================
    // 2. STOP OLD JAZZY SIM CONTAINER IF IT EXISTS
    // ========================================================

    std::cout << "[2] Stopping old Jazzy simulator container...\n";

    std::system(
        "docker stop ros2-jazzy-sim "
        ">/dev/null 2>&1 || true"
    );

    // ========================================================
    // 3. START HUMBLE DEVELOPMENT CONTAINER
    // ========================================================

    std::cout
        << "[3] Starting Humble container: "
        << containerName
        << "\n";

    std::string startContainer =
        "docker start " +
        shellQuote(containerName) +
        " >/dev/null 2>&1";

    if (std::system(startContainer.c_str()) != 0)
    {
        std::cerr
            << "\nERROR: Could not start container:\n"
            << "    "
            << containerName
            << "\n";

        return 1;
    }

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );

    // ========================================================
    // 4. VERIFY ROS HUMBLE
    // ========================================================

    std::cout << "[4] Verifying ROS 2 Humble...\n";

    std::string verifyRos =
        "docker exec " +
        shellQuote(containerName) +
        " test -f /opt/ros/humble/setup.bash";

    if (std::system(verifyRos.c_str()) != 0)
    {
        std::cerr
            << "\nERROR: ROS 2 Humble not found inside container.\n";

        return 1;
    }

    std::cout << "    ROS 2 Humble found.\n";

    // ========================================================
    // 5. VERIFY WORKSPACE
    // ========================================================

    std::cout << "[5] Verifying quadruped workspace...\n";

    const std::string packageXml =
        workspace +
        "/src/" +
        packageName +
        "/package.xml";

    std::string verifyWorkspace =
        "docker exec " +
        shellQuote(containerName) +
        " test -f " +
        shellQuote(packageXml);

    if (std::system(verifyWorkspace.c_str()) != 0)
    {
        std::cerr
            << "\nERROR: Could not find:\n"
            << "    "
            << packageXml
            << "\n";

        return 1;
    }

    std::cout
        << "    Found "
        << packageXml
        << "\n";

    // ========================================================
    // 6. VERIFY HARMONIC-COMPATIBLE ROS_GZ
    // ========================================================

    std::cout
        << "[6] Checking Humble / Harmonic ros_gz...\n";

    std::string verifyRosGz =
        "docker exec " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(
            "source /opt/ros/humble/setup.bash; "
            "ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1"
        );

    if (std::system(verifyRosGz.c_str()) != 0)
    {
        std::cerr
            << "\nERROR: ros_gz_bridge is not installed.\n\n"
            << "For ROS 2 Humble + Gazebo Harmonic install:\n\n"
            << "    ros-humble-ros-gzharmonic\n\n"
            << "inside the Humble container.\n";

        return 1;
    }

    // Check specifically for the Harmonic compatibility package.
    std::string harmonicPackage =
        captureCommand(
            "docker exec " +
            shellQuote(containerName) +
            " bash -lc " +
            shellQuote(
                "dpkg-query -W -f='${Status}' "
                "ros-humble-ros-gzharmonic "
                "2>/dev/null || true"
            )
        );

    if (harmonicPackage.find("install ok installed")
        == std::string::npos)
    {
        std::cerr
            << "\nWARNING:\n"
            << "ros_gz_bridge exists, but I could not confirm\n"
            << "ros-humble-ros-gzharmonic is installed.\n\n"
            << "The normal Humble ros_gz packages target "
            << "Gazebo Fortress.\n"
            << "Your Mac is running Gazebo Harmonic.\n\n";
    }
    else
    {
        std::cout
            << "    ros-humble-ros-gzharmonic found.\n";
    }

    // ========================================================
    // 7. FIND MAC NETWORK INTERFACE + IP
    // ========================================================

    std::cout
        << "[7] Detecting Mac network address...\n";

    std::string macInterface =
        captureCommand(
            "route -n get default 2>/dev/null "
            "| awk '/interface:/{print $2}'"
        );

    if (macInterface.empty())
    {
        std::cerr
            << "ERROR: Could not determine Mac interface.\n";

        return 1;
    }

    std::string macIp =
        captureCommand(
            "ipconfig getifaddr " +
            macInterface
        );

    if (macIp.empty())
    {
        std::cerr
            << "ERROR: Could not determine Mac IP.\n";

        return 1;
    }

    std::cout
        << "    Interface : "
        << macInterface
        << "\n"
        << "    Mac IP    : "
        << macIp
        << "\n";

    // ========================================================
    // 8. FIND CONTAINER IP
    // ========================================================

    std::cout
        << "[8] Detecting container IP...\n";

    std::string containerIp =
        captureCommand(
            "docker inspect -f "
            "'{{range.NetworkSettings.Networks}}"
            "{{.IPAddress}}"
            "{{end}}' "
            + shellQuote(containerName)
        );

    if (containerIp.empty())
    {
        std::cerr
            << "ERROR: Could not determine container IP.\n";

        return 1;
    }

    std::cout
        << "    Container IP : "
        << containerIp
        << "\n"
        << "    Partition    : "
        << partition
        << "\n";

    // ========================================================
    // 9. CLEAN OLD ROS PROCESSES
    // ========================================================

    std::cout
        << "[9] Cleaning old ROS simulation processes...\n";

    std::string cleanupInsideContainer =
        "pkill -TERM -f '[p]arameter_bridge' "
        ">/dev/null 2>&1 || true; "

        "pkill -TERM -f '[r]os2 topic echo /clock' "
        ">/dev/null 2>&1 || true; "

        "pkill -TERM -f '[q]uadruped' "
        ">/dev/null 2>&1 || true; "

        "sleep 1";

    std::string cleanupCommand =
        "docker exec " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(cleanupInsideContainer);

    std::system(cleanupCommand.c_str());

    // ========================================================
    // 10. START GAZEBO HARMONIC SERVER ON MAC
    // ========================================================

    std::cout
        << "[10] Starting Gazebo Harmonic server...\n";

    std::string gazeboServer =
        "clear; "

        "echo '================================'; "
        "echo ' GAZEBO HARMONIC SERVER'; "
        "echo '================================'; "

        "export GZ_IP=" +
        macIp +
        "; "

        "export GZ_PARTITION=" +
        partition +
        "; "

        "echo \"GZ_IP=$GZ_IP\"; "
        "echo \"GZ_PARTITION=$GZ_PARTITION\"; "
        "echo ''; "

        "exec gz sim -s shapes.sdf";

    runMacTerminalCommand(
        gazeboServer
    );

    std::this_thread::sleep_for(
        std::chrono::seconds(3)
    );

    // ========================================================
    // 11. START GAZEBO GUI ON MAC
    // ========================================================

    std::cout
        << "[11] Starting Gazebo GUI...\n";

    std::string gazeboGui =
        "clear; "

        "echo '================================'; "
        "echo ' GAZEBO HARMONIC GUI'; "
        "echo '================================'; "

        "export GZ_IP=" +
        macIp +
        "; "

        "export GZ_PARTITION=" +
        partition +
        "; "

        "echo \"GZ_IP=$GZ_IP\"; "
        "echo ''; "

        "exec gz sim -g";

    runMacTerminalCommand(
        gazeboGui
    );

    std::this_thread::sleep_for(
        std::chrono::seconds(3)
    );

    // ========================================================
    // 12. START HUMBLE <-> HARMONIC BRIDGE
    // ========================================================

    std::cout
        << "[12] Starting Humble / Harmonic bridge...\n";

    std::string bridgeInsideContainer =
        "clear; "

        "source /opt/ros/humble/setup.bash; "

        "export GZ_IP=" +
        containerIp +
        "; "

        "export GZ_RELAY=" +
        macIp +
        "; "

        "export GZ_PARTITION=" +
        partition +
        "; "

        "echo '================================'; "
        "echo ' HUMBLE <-> HARMONIC BRIDGE'; "
        "echo '================================'; "

        "echo \"GZ_IP=$GZ_IP\"; "
        "echo \"GZ_RELAY=$GZ_RELAY\"; "
        "echo \"GZ_PARTITION=$GZ_PARTITION\"; "
        "echo ''; "

        "exec ros2 run ros_gz_bridge "
        "parameter_bridge "
        "\"/clock@rosgraph_msgs/msg/Clock"
        "[gz.msgs.Clock\"";

    std::string bridgeCommand =
        "docker exec -it " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(bridgeInsideContainer);

    runMacTerminalCommand(
        bridgeCommand
    );

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );

    // ========================================================
    // 13. BUILD + RUN QUADRUPED
    // ========================================================

    std::cout
        << "[13] Building and starting quadruped...\n";

    std::string quadrupedInsideContainer =
        "clear; "

        "source /opt/ros/humble/setup.bash; "

        "cd " +
        workspace +
        "; "

        "echo '================================'; "
        "echo ' QUADRUPED CONTROL'; "
        "echo '================================'; "
        "echo ''; "

        "echo 'Building quadruped_control...'; "

        "colcon build "
        "--symlink-install "
        "--packages-select quadruped_control; "

        "BUILD_RESULT=$?; "

        "if [ $BUILD_RESULT -ne 0 ]; then "
        "echo ''; "
        "echo 'BUILD FAILED'; "
        "exec bash; "
        "fi; "

        "source /workspace/install/setup.bash; "

        "echo ''; "
        "echo 'Build successful.'; "
        "echo 'Starting quadruped node...'; "
        "echo ''; "

        "exec ros2 run "
        "quadruped_control "
        "quadruped";

    std::string quadrupedCommand =
        "docker exec -it " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(quadrupedInsideContainer);

    runMacTerminalCommand(
        quadrupedCommand
    );

    std::this_thread::sleep_for(
        std::chrono::seconds(3)
    );

    // ========================================================
    // 14. GAZEBO TRANSPORT DEBUG TERMINAL
    // ========================================================

    std::cout
        << "[14] Starting Gazebo topic monitor...\n";

    std::string gzDebugInsideContainer =
        "clear; "

        "source /opt/ros/humble/setup.bash; "

        "export GZ_IP=" +
        containerIp +
        "; "

        "export GZ_RELAY=" +
        macIp +
        "; "

        "export GZ_PARTITION=" +
        partition +
        "; "

        "echo '================================'; "
        "echo ' GAZEBO TRANSPORT TOPICS'; "
        "echo '================================'; "
        "echo ''; "

        "echo 'Gazebo topics visible from Docker:'; "
        "echo ''; "

        "gz topic -l; "

        "echo ''; "
        "echo 'Press Enter to refresh...'; "

        "while read line; do "
        "clear; "
        "echo 'Gazebo topics visible from Docker:'; "
        "gz topic -l; "
        "done";

    std::string gzDebugCommand =
        "docker exec -it " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(gzDebugInsideContainer);

    runMacTerminalCommand(
        gzDebugCommand
    );

    // ========================================================
    // 15. ROS /CLOCK MONITOR
    // ========================================================

    std::this_thread::sleep_for(
        std::chrono::seconds(2)
    );

    std::cout
        << "[15] Starting /clock monitor...\n";

    std::string clockInsideContainer =
        "clear; "

        "source /opt/ros/humble/setup.bash; "

        "source /workspace/install/setup.bash "
        "2>/dev/null || true; "

        "echo '================================'; "
        "echo ' ROS /clock MONITOR'; "
        "echo '================================'; "
        "echo ''; "

        "echo 'Waiting for Gazebo simulation time...'; "
        "echo ''; "

        "exec ros2 topic echo /clock";

    std::string clockCommand =
        "docker exec -it " +
        shellQuote(containerName) +
        " bash -lc " +
        shellQuote(clockInsideContainer);

    runMacTerminalCommand(
        clockCommand
    );

    // ========================================================
    // FINISHED
    // ========================================================

    std::cout
        << "\n========================================\n"
        << " Simulation environment launched\n"
        << "========================================\n\n"

        << "ROS:           Humble\n"
        << "Gazebo:        Harmonic\n"
        << "Container:     "
        << containerName
        << "\n"

        << "Mac IP:        "
        << macIp
        << "\n"

        << "Container IP:  "
        << containerIp
        << "\n"

        << "GZ partition:  "
        << partition
        << "\n\n"

        << "Started:\n"
        << "  [x] Gazebo Harmonic server\n"
        << "  [x] Gazebo GUI\n"
        << "  [x] Humble/Harmonic ros_gz_bridge\n"
        << "  [x] quadruped_control build\n"
        << "  [x] quadruped node\n"
        << "  [x] Gazebo topic monitor\n"
        << "  [x] ROS /clock monitor\n\n";

    return 0;
}