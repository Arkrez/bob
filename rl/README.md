# Learning to walk

Rocky-Bob learns to walk with reinforcement learning. Training happens in MuJoCo, using the PPO algorithm from Stable-Baselines3. The trained policy then runs in Webots over ROS 2, and later can run on the real robot.

MuJoCo is used for training because it runs natively on Apple Silicon. It can simulate thousands of steps per second on each CPU core, with 12 robots training in parallel. Webots trains at roughly real time, so a run that takes about an hour here would take days there.

## Setup

```bash
cd rl
python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python make_mjcf.py
```

`make_mjcf.py` builds the MuJoCo model, `rocky_bob.xml`, from `cad/rocky_bob/sim/rocky_bob.urdf`. Re-run it whenever you change the CAD, so training always matches the robot.

## Train

```bash
.venv/bin/python train.py                  # 30M steps on all cores, about 1 hour on an M2 Max
.venv/bin/tensorboard --logdir runs        # learning curves at http://localhost:6006 (ep_rew_mean should climb)
.venv/bin/mjpython watch.py --live         # live 3D window of a robot that is training right now
.venv/bin/mjpython watch.py runs/walk      # or: the newest saved policy on rough ground with shoves (--terrain 0..1)
.venv/bin/python train.py --resume runs/walk/latest.zip --steps 20e6   # keep going
```

Each run writes to `runs/<name>/`:

- `latest.zip`: the trained model.
- `vecnormalize.pkl`: input scaling saved with the model.
- `checkpoints/`: snapshots taken during training.
- `policy.npz`: the deployable policy. It needs only numpy.

## Watch it

```bash
.venv/bin/python play.py runs/walk --video walk.mp4 --cmd 0.2 0 0   # vx, vy (m/s), yaw rate (rad/s)
.venv/bin/mjpython play.py runs/walk --viewer                       # live 3D window
```

## Run it in Webots

From the repo root:

```bash
scripts/start_sim.sh --policy rl/policies/rocky_walk.npz
```

Then drive it:

```bash
scripts/sim_ros.sh ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
```

To score it on the same four tests as MuJoCo:

```bash
scripts/sim_ros.sh python3 /workspace/rl/webots_eval.py
```

In the sim, the Webots bridge runs the policy itself, stepped by simulation time. The setup matches training:

- **Timing:** a policy step every 20 ms of simulated time (5 × 4 ms physics steps), with a gait clock that also runs on sim time. That holds however fast Webots runs; here, under Docker emulation, it runs at about 0.9× real time.
- **Servo model:** the same MG995 model as MuJoCo: torque = KP × (target − angle) − KV × speed, clamped to 0.94 N·m. Webots' own position controller is much stiffer.
- **Joints and ground:** the same joint damping and friction, and the same 0.9 foot friction. All of these come from `spec.py`.
- **Servo rotor inertia:** MuJoCo models the servo's rotor as joint armature (0.003 kg·m²), and Webots has nothing equivalent. `build.py` therefore adds it to each child leg link's inertia about its joint axis.
  - Without it, the knee link is so light that the servo's damping, applied once per 4 ms physics step, becomes unstable.
  - That matters for the 25 kg servo's stiffer gains: the legs chattered and the body wobbled at 2.0–2.3 rad/s, against 0.4 in MuJoCo.
  - With it, Webots matches MuJoCo: it walks at 0.198 against 0.197 m/s, and the body wobbles at 0.3–0.5 rad/s.

The bridge also publishes `/clock` (sim time) and `/sim/position` (ground-truth position). It stops the robot if `/cmd_vel` is silent for one second of sim time.

At first I drove Webots from a separate ROS node running on the wall clock, with Webots' stock motors. That reached only about a fifth of MuJoCo's speed. With the changes above (`rocky_walk.npz`, flat ground, no shoves):

| test | commanded | MuJoCo | Webots |
| --- | --- | --- | --- |
| forward | 0.15 m/s | 0.158 | 0.135 |
| backward | −0.10 m/s | −0.057 | −0.057 |
| turn in place | 0.40 rad/s | 0.28 | 0.29 |
| sidestep | 0.08 m/s | 0.057 | 0.055 |

The remaining forward gap, about 15%, is likely from differences between the two contact models and from motor armature, which Webots doesn't simulate.

`ros_policy_node.py` is for the real robot. It reads `/imu` and `/joint_states`, takes `/cmd_vel`, and publishes the 12 joint targets 50 times a second, timed by the wall clock, which is fine on hardware running in real time. With `--open-loop` it feeds back its own last commands in place of measured angles, because MG995s can't report their angle.

## Walking to a target with the Pi camera

The walker only follows velocity commands, and it drifts: told to go straight for 10 s, it curves about 27° left. Goal-seeking is a separate, small **navigator** layered on top. This is the usual way navigation is added to legged robots, and the walker isn't retrained.

**How the two levels split the work:**

| | runs at | input | output |
| --- | --- | --- | --- |
| walker (`policies/rocky_walk.npz`, frozen) | 50 Hz | IMU, joints, velocity command | 12 joint targets |
| navigator | 5 Hz | what the camera has seen of the target | velocity command |

**The default navigator is scripted, not trained** (`nav_spec.ScriptedNavigator`, `--nav scripted`). It works like this:

1. Spin until the camera sees the target.
2. Turn in place until the target is within 14° of straight ahead.
3. Go straight, steering gently: run if the target is more than 1 m away, walk if it's closer.
4. If a full turn passes without a sighting, run forward about 1.4 m and look again.

**Why scripted:** with the walk+run walker it beat every trained navigator I tried. Over 20 one-minute runs: 4.1 targets/min and 94% path efficiency, against 3.1 and 85% for the best trained one. It also arcs only about 5% of the time. The trained navigator (`policies/rocky_nav.npz`, `nav_v3`) learned turn-then-walk too, but it loses sight of the target more often.

**Webots:** in an 8 × 8 m arena it reached 5 of 6 targets in 13–20 s each. It ran for about 45% of the time it was moving and arced 6–9%. The miss was 3.9 m away, beyond the camera's 2.5 m range.

**Limit:** a single forward camera can't find a target it has never seen if it's out of range. Exploring is then a random walk.

**What the navigator gets.** It only gets what the Pi camera would give it: the last detected position of the target, whether it's in view right now, and how old that sighting is. `camera.py` models a Camera Module 3 on the nose of the rock:

- 66° field of view, 0.15–2.5 m range
- about 10 detections per second, roughly 0.1 s late, with 10% missed
- noisy distance and bearing

Targets often start behind it, so the navigator learned to turn and look.

**Rewards** (`nav_env.py`):

- *Earned:* getting closer, reaching the target, and facing the target while walking forward.
- *Penalised:* sideways commands, jerky commands, and time.
- *Turn first, then walk straight:*
  - moving forward while not facing the target costs speed × (1 − cos(bearing))
  - turning while moving forward costs turn rate × speed

  So it turns in place until it's lined up, then walks, or runs when the target is far.

The walker keeps its terrain, shoves and randomisation while the navigator trains, so the navigator learns to correct its drift.

**Commands:**

```bash
.venv/bin/python nav_train.py                                   # trains on top of the frozen walker
.venv/bin/python nav_eval.py policies/rocky_nav.npz             # goals/min, time to goal, path efficiency
.venv/bin/python nav_eval.py baseline                           # hand-written spin-then-steer controller
.venv/bin/mjpython watch.py --nav policies/rocky_nav.npz        # 3D view, random orange targets (trained navigator)
```

**In Webots,** from the repo root:

```bash
scripts/start_sim.sh --nav scripted
```

Then drag the orange post around, or move it from the command line:

```bash
scripts/sim_ros.sh ros2 topic pub --once /goal geometry_msgs/msg/PointStamped "{point: {x: -1.0, y: 0.5}}"
```

`/nav/status` reports what it's doing. The bridge reads the post's true position, runs it through the same camera model, and steers the walker.

**On the real robot** (untested, I have no hardware to try it on):

```bash
python3 rl/camera_target_node.py --marker-size 0.08
```

```bash
python3 rl/ros_policy_node.py rl/policies/rocky_walk.npz --open-loop --nav scripted
```

- `camera_target_node.py` uses Picamera2 and OpenCV to find ArUco marker 0 (`DICT_4X4_50`) and publishes `/target`.
- For the target, print a marker from that dictionary and mount it on a post about 0.25 m high, which is camera height when Bob stands. Measure the black square for `--marker-size`.
- Calibrating the camera with `--calibration` makes the distance estimates more accurate.

## Training speed

Measured on an M2 Max (8 performance and 4 efficiency cores):

| | steps per second |
| --- | --- |
| one robot, one core, 2 ms physics (original) | 2,400 |
| one robot, one core, 4 ms physics + vectorised rewards | 3,000 |
| full PPO training, 12 processes, 2 ms (walk_steady) | 6,900 |
| full PPO training, 12 workers × 4 robots, 4 ms | 7,300–7,900 |

**What changed:**
- **4 ms physics step** (`spec.PHYSICS_DT`): the walker performs the same as at 2 ms, physics cost halves, and it now matches Webots.
- **Vectorised rewards:** contacts, terrain heights and body rotation are each computed once per step, with array operations instead of Python loops.
- **Batched workers** (`vec.py`): each worker process steps 4 robots and sends one message, instead of one robot per process.

**What limits it now:** the single main process that gathers every result and runs PPO. Adding workers stops helping after about 8, and the Webots demo plus Docker take about 2 cores when they're running. Close Webots while training. PyTorch on the Apple GPU is slower than the CPU for networks this small.

**For a much bigger speed-up,** the simulation has to stop going through Python every step. Options:
- **MuJoCo Playground / MJX:** MuJoCo in JAX on an NVIDIA GPU. Thousands of robots run in parallel, typically 100× or more faster. This means porting the env and rewards to JAX; the MJCF model can be reused as is. It's best on a Linux machine or cloud GPU, because JAX on Apple Silicon is experimental.
- **Isaac Lab** on an NVIDIA GPU: same idea, using the URDF.

**For Webots** (testing, not training): it runs at about 0.9× real time because the bridge runs under emulation in Docker. Starting Webots with `--mode=fast` and closing the 3D view helps; a native macOS controller would help more.

## What the policy sees and does

The shared definitions are in `spec.py`, which keeps training and deployment identical.

- **Inputs:**
  - gravity direction and gyro, from an IMU
  - the velocity command
  - the 12 joint angles
  - its last action
  - a gait clock
- **Walk and run:** the gait clock input ticks at a rhythm set by the commanded speed: a 0.6 s stride up to 0.18 m/s, quickening to 0.4 s at 0.35 m/s (`spec.gait_period`). "Run" is just a faster command. The Webots bridge and the robot node advance the clock the same way.
- **Outputs:** 12 joint-angle offsets from the standing pose (hip 0.75 rad, knee 1.05 rad), sent at 50 Hz. Targets stay within 90% of each servo's range, and the full action range sits inside the soft limits. A policy that can push a joint onto its end stop learns to lean on it, and on an MG995 that strips the gears.
- **Rewards:**
  - *Encouraged:* matching the commanded speed and turn rate, staying level at standing height, and taking clean steps with the feet in the air.
  - *Discouraged:* wasted power, jerky actions, slipping feet, and legs dragging on the ground.
  - An episode ends if the robot tips past 60° or its belly hits the floor.
- **Smoothness:** this uses the standard `legged_gym` set of penalties (Rudin et al., *Learning to Walk in Minutes*):
  - action rate, the 1st difference of the actions
  - action smoothness, the 2nd difference
  - joint acceleration
  - joint velocity

  Each weight is set to cost about 5% of a good walk's reward, so jitter costs more than it gains. `progress.md` tracks the result as two numbers: joint direction reversals per second (a clean trot is about 3) and joint travel per metre walked.
- **Servo protection:** penalties for joints within 15% of their limit and for torque above 75% of stall (0.7 N·m). Both follow `legged_gym`'s soft-limit terms.
- **Stepping, not dragging:**
  - air time on each step is rewarded
  - a foot sliding while on the ground is penalised
  - a foot moving while lower than 3 cm above the ground beneath it is penalised (Isaac Lab's `foot_clearance`)
- **Penalty curriculum:** the shaping penalties (smoothness, servo protection, power, foot drag) start at 25% and ramp to full over the first 10M steps. Without that, an early policy discovers that freezing is the cheapest option.
- **Turning in place:**
  - a bonus for keeping the body over the spot where the turn began
  - a small cost per footfall and for the body sliding
  - a slower 1.0 s stride (`spec.TURN_PERIOD`), so it pivots with fewer, bigger steps

  `turn_eval.py` measures footfalls per radian and drift.
- **Stable body, the "chicken head":** two bonuses that peak when the body, and so the camera, stays still while the legs work:
  - *steady:* low roll and pitch rate
  - *smooth ride:* low body acceleration over each 20 ms step

  `progress.md` tracks both.
- **Clean, even steps:**
  - a 4.5 cm target step height for every leg
  - penalties when the legs' step heights differ and for bounces (quick re-touches)
  - extra penalties for dragging feet during pivots

  `step_eval.py` measures them.
- **Rough terrain and shoves** (`terrain.py`), for dynamic balance:
  - The ground has smooth bumps up to 4 cm and up to 400 blocks up to 7 cm tall (raised from 4.5 cm for the 25 kg servos), on a 12 × 12 m height field. The spawn area is a flat pad.
  - Each training robot moves up a level after walking at least 60% of its commanded distance, and down a level after a fall or a stall.
  - Every 2–5 s the robot gets a shove of up to 0.5 m/s plus a twist.
  - The policy is blind: it feels the ground through its legs and IMU, like the real robot.
  - TensorBoard plots `curriculum/terrain_level` and `curriculum/penalty_scale`.
- **Randomisation:** body mass +30/−20% (the PSU weight isn't known yet), leg masses ±10%, friction, servo stiffness and strength, and sensor noise. This keeps the policy from relying on exact simulator details.

## Moving to the real robot

- **Add an IMU.** The policy needs gravity direction and gyro rates. A BNO085 or MPU-6050 on the Pi's I2C bus works. Publish it on `/imu`.
- **Run open loop.** MG995s can't report their angle, so run with `python3 rl/ros_policy_node.py rl/runs/walk/policy.npz --open-loop`. The node then feeds its own last commands back in place of measured angles. Set the servo bridge's joint directions and offsets first, using `/jdirection` and `/joint_offset`.
- **Expect a gap.** Real servos have backlash, delay and droop. Start with the robot held up off the ground, use small commands, and stay within reach of the power switch.

## Limits

- **Standing up:** the robot can't push itself up from lying on its belly, because the hip servos stall. Training therefore always starts from standing, and Webots spawns it standing.
- **Weight:** standing takes about 21% of the MG995's stall torque. Walking on rough ground takes more, and the torque-limit penalty keeps it below 75%. Lighter prints or stronger servos give the gait more headroom.
