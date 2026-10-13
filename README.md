# Bob — Motorcycle Chain Lubrication Robot

Owning a chain-driven motorcycle means periodically lubricating the drive chain. It is a repetitive maintenance task, and I wanted to automate it away. **Bob** is the quadruped robot I am building to do that.

The goal is for Bob to move around the motorcycle, get into position near the drive chain, identify where lubricant needs to be applied, and eventually perform the lubrication process automatically.

<img width="1215" height="749" alt="Bob quadruped robot" src="https://github.com/user-attachments/assets/0fdad499-c6f1-402a-a0a8-6c3416fa759d" />

## What Bob Needs to Do

To solve the original maintenance problem, Bob needs several capabilities:

- Walk and position himself reliably around a motorcycle
- Control each leg accurately enough for close positioning
- Locate the motorcycle and drive-chain area using vision
- Carry and position a chain-lubrication mechanism
- Coordinate movement, perception, and hardware through ROS 2
- Test new behavior in simulation before running it on the physical robot

## Hardware

### Raspberry Pi 5

The Raspberry Pi 5 is Bob's main computer. It runs ROS 2 and the higher-level robot software, including locomotion control, hardware communication, and camera processing.

### Pimoroni Servo 2040

The Servo 2040 handles low-level control of the leg servos. The Raspberry Pi sends commands through the ROS 2 hardware bridge, and the Servo 2040 generates the PWM signals used to position the servos.

### 16 × MG995 Servos

Bob has four legs with four servos per leg. These provide the joint movement used for stance control, walking, and positioning.

### Power System

Bob uses an onboard battery and power-distribution setup so the Raspberry Pi, controller electronics, and servo power system can operate without being tethered to a bench power supply.

### Camera

The camera provides visual input for perception work. The long-term goal is to use vision to help Bob navigate around the motorcycle and locate the drive-chain area accurately enough to perform the maintenance task.

## Software and Architecture

### ROS 2 Humble

ROS 2 is the communication layer between Bob's software components. It lets the locomotion controller, hardware bridge, camera pipeline, and future perception and task-planning systems communicate as separate parts of the robot.

The main ROS 2 executable currently starts:

- `QuadrupedControllerNode` — gait generation, inverse kinematics, joint targets, and robot movement
- `Servo2040BridgeNode` — converts ROS 2 commands into communication with the physical Servo 2040
- `CameraDisplayNode` — receives and processes camera images

### Webots

Webots is the main simulation environment for Bob. It provides a place to test joint motion, inverse kinematics, gait logic, and ROS 2 integration before sending the same behavior to the physical robot.

That makes it possible to iterate on movement without repeatedly risking the servos, frame, or surrounding hardware.

The current Webots world is:

```text
webots_quadruped/worlds/quadruped.wbt
```

### C++

Most of Bob's control software is written in C++. It is used for the ROS 2 nodes, locomotion logic, inverse kinematics, hardware communication, and supporting robot-control code.

### OpenCV / cv_bridge

OpenCV and `cv_bridge` are used for camera handling and provide the foundation for future visual detection and positioning around the motorcycle.

### Cyclone DDS

Cyclone DDS is used as the ROS 2 middleware configuration for communication between the development environment, containers, and robot hardware.

## Motion Control

Bob's locomotion system is being built around custom leg kinematics and gait generation.

Current work includes:

- Joint-angle control
- Per-leg inverse kinematics
- Coordinate transforms between robot and leg frames
- Servo-direction and angle mapping
- Multi-leg gait sequencing
- Simulation-to-hardware control flow

The goal is to make Bob stable and predictable enough that locomotion becomes a reliable tool for the chain-lubrication task rather than a separate experiment.

## Repository Layout

```text
bob/
├── src/
│   ├── quadruped_control/     # Main ROS 2 C++ control package
│   └── quadruped_sim/         # Gazebo / simulation experiments
├── webots_quadruped/
│   ├── protos/                # Webots robot PROTO files
│   └── worlds/
│       └── quadruped.wbt      # Main Webots world
├── voice_control/             # Voice-control experiments
├── Quadruped.proto            # Webots robot model
├── quadruped_webots.urdf      # Robot description used with Webots
├── cyclonedds.xml             # ROS 2 DDS configuration
└── README.md
```

## Building the ROS 2 Workspace

From the repository root inside a ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the Controller

After building and sourcing the workspace:

```bash
ros2 run quadruped_control quadruped
```

This starts the quadruped controller, Servo 2040 bridge, and camera node.

## Current Status

Bob is still under active development. The current focus is making locomotion and simulation reliable, improving the connection between ROS 2 and the physical robot, and building the perception and positioning capabilities needed for the motorcycle chain-lubrication task.

The end result I am working toward is simple: take a maintenance job I have to do repeatedly on my motorcycle and have Bob do it for me.
