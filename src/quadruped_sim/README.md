# quadruped_sim

Gazebo **Harmonic** simulation package (URDF/xacro, controllers, world, launch)
built to match the kinematic chain in `QuadrupedControllerNode.cpp`:

```
base_link --(fixed, z=HIP_Z)--> hip_mast --(jointN yaw, RotZ)--> link1
        --(jointN3 pitch, RotX -1)--> link2 --(jointN4 pitch, RotX -1)--> link3 --foot
```

## Assumptions baked in (placeholders — replace when possible)

- Units: your numbers (2 in link length, 1 in width, 2 in offsets) converted to meters.
- `HIP_Z = 4.5 in` taken directly from `QuadrupedControllerNode.cpp`.
- Body chassis size (6in x 6in x 1.5in) and all masses/inertias are **guesses** —
  not in the code you shared. Physically the legs will look/move right, but sim
  dynamics (how it balances, how heavy it feels) won't be accurate until you
  supply real body dimensions and masses.
- Leg quadrant signs (`{+,-,+,-}`) assigned as: leg0 front-right, leg1
  front-left, leg2 rear-right, leg3 rear-left. Swap the `x_sign`/`y_sign`
  arguments in the xacro if your leg numbering differs.
- Joint limits set to ±90° to match the code's servo clamp to [0,180] centered
  on 90.

## Build & run

```bash
cd ~/ros2_ws/src
# copy this quadruped_sim/ folder in here
cd ~/ros2_ws
colcon build --packages-select quadruped_sim
source install/setup.bash
ros2 launch quadruped_sim gazebo_sim.launch.py
```

This starts Gazebo Harmonic, spawns the robot, and brings up
`joint_state_broadcaster` + `leg_position_controller`
(`position_controllers/JointGroupPositionController` over all 12 joints).

## Important gap: your controller doesn't talk to ros2_control yet

`QuadrupedControllerNode` publishes a **custom comma-separated string** on
`/servo_commands` (servo degrees, 0-180) — it does not publish to
`leg_position_controller/commands` (`std_msgs/Float64MultiArray`, radians,
one value per joint in the order listed in
`config/quadruped_controllers.yaml`).

To actually drive this sim from your existing node you need a small bridge
that either:

1. Subscribes to `/servo_commands`, converts each servo degree back to
   radians (`(servoDeg - 90) * DEG_TO_RAD`, undoing `applyDirection`/
   `toServoDegJx`), and publishes the 12-value array to
   `/leg_position_controller/commands`; or
2. Have `QuadrupedControllerNode` publish `sensor_msgs/JointState` /
   `trajectory_msgs/JointTrajectory` directly instead of the custom string,
   and swap `leg_position_controller` for a
   `joint_trajectory_controller/JointTrajectoryController`.

Happy to write that bridge node (or the JointState-based refactor) next —
let me know which direction you'd rather go, and paste
`QuadrupedControllerNode.hpp` if you can, so the leg geometry above can be
made exact instead of placeholder.
