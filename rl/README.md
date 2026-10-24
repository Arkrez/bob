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

### Training more, on demand

```bash
scripts/train_more.sh               # 2 hours of rounds in the background, then it stops
scripts/train_more.sh --hours 6     # longer (overnight, say), or --rounds 3
scripts/train_more.sh --status      # champions so far and the recent log
scripts/train_more.sh --stop        # finish the current round, then stop early
```

`train_rounds.py` runs rounds until its budget is used. It never stops mid-round, and the Mac is kept awake only while it runs. The champions and the round counter live in `runs/rounds/`, so each session carries on where the last one stopped.

Each round trains the walker 10M steps (about 30 minutes) on the full curriculum. `keep_best.py` scores every 1M-step snapshot against the champion. A snapshot that beats it, without falling or losing more than 10% of walking or top speed, becomes the new champion and is installed as `policies/rocky_walk.npz`. A running sim picks it up by itself.

Training carries on from where the last round ended (the "frontier"), so a new skill such as stairs can build up over several rounds even while it costs speed. After 3 rounds with no new champion it restarts from the champion. Every second round it also trains the lidar navigator 1M steps, scores it over 20 one-minute runs among obstacles (goals/min minus bumps and falls), and installs a winner as `policies/rocky_nav_lidar.npz`. Every third round it trains self-righting 5M steps and installs a winner as `policies/rocky_recover.npz`.

Each round deletes its losing snapshots, videos and checkpoints, and keeps the score tables. Old champions stay in `policies/history/`. A session pauses if the disk gets below 8 GB free, and waits for any training you started by hand to finish first. A crashed round is logged in `runs/rounds/log.md` and skipped.

### Self-righting

`recover_env.py` drops him on his back (half the time), on a side, or at any angle, from a random leg pose, then gives him 8 s to roll over and stand up. It's a separate policy from the walker:

- **Joint range:** it uses nearly the whole range (`spec.recovery_targets`). Rolling a rock needs far bigger swings than walking.
- **Rewards:** being upright, then standing height and the standing pose, then each second spent fully up. The pose term is linear as well as an exponential. With only the exponential, the first 25M steps learned to roll over and rise to standing height, but ended with the legs far from the standing pose (pose error ~13, against the 0.3 needed to hand back to the walker), because the exponential gave no pull from that far away.
- **Smooth and controlled:** the first version flung itself over, with a typical body spin of 7.9 rad/s and 20 rad/s at worst. Now joint targets move no faster than 3 rad/s (`spec.RECOVERY_MAX_RATE`, in training and on the robot). Spinning faster than 2 rad/s costs steeply, and so does the rock hitting the ground at over 30 N. With the speed limit alone, the old policy rolled over but couldn't get its legs under itself in time and landed on its belly (up to 3,400 N). Retraining under the limit is fixing that. `recover_eval.py` reports body spin and impact peaks.
- **Gentle on the lidar:** upside down he rests on the RPLIDAR C1, the highest point of the robot, and there is no roll guard. Every second the lidar is on the ground costs 4 points, more than twice what being upright earns. On top of that, each newton on it costs, hard knocks (over 30 N) cost more, and so does each new touch. Spinning, fast joints, servo strain and jerky actions also cost.

```bash
.venv/bin/python train.py --task recover --name recover --steps 40e6 --penalty-start 1.0
.venv/bin/python recover_eval.py runs/recover/policy.npz   # % recovered, time to stand, lidar time and peak force
```

**How he runs it** (`recover_spec.Recovery`, shared by `recover_eval.py` and the Webots bridge): the learned policy rolls him upright, in about a second from his back. It always ends in a stable splayed stance, with three hip-yaw joints at their limit, which the walker can't start from, and training couldn't pull him out of it (swinging a loaded leg drags its foot). So once he's upright and steady, a scripted tidy-up moves one leg at a time while the other three carry him: lift the foot, swing the yaw home, put it down at the standing pose. If he tips over on the way, it goes back to rolling.

| (`recover_eval.py`, 24 falls, 12 s each) | back to standing | median time | lidar on the ground | lidar peak force |
| --- | --- | --- | --- | --- |
| recover 10M | 46% | 6.7 s | 0.0 s | 0 N |
| **recover 40M** (installed as `policies/rocky_recover.npz`) | **83%** | **6.7 s** | **0.0 s** | **0 N** |

In Webots the bridge switches to self-righting when he tips past 60° for 0.3 s, and back to walking when the tidy-up is done. Navigation pauses meanwhile. Flipped by `flip_test.py`, he was walking again 7.1 s after 180°, 7.4 s after 150° and 11.8 s after 90° (on his side). To flip him by hand:

```bash
scripts/sim_ros.sh ros2 topic pub --once /flip std_msgs/msg/Float64 "{data: 180.0}"
```

`train_more.sh` sessions train self-righting further every third round, and install only improvements.

To model the lidar properly the collision shapes changed. The body is now the rock's own convex shell, so it rolls like the real one, plus boxes for the plinth and the lidar, named `lidar` so its contacts can be measured. Dropped upside down with no control, he rests on the lidar alone: 19 N on average, up to 61 N.

### Stairs and step-ups

At higher curriculum levels the training ground (`terrain.py`) also has stairs of five kinds and wide flat platforms, on top of the bumps and blocks ([preview](stairs_terrain.png)):

- **Pyramids:** steps up on every side to a landing.
- **Straight flights:** up, a landing, and half the time down again, 0.8–2.5 m wide. You can walk off the side of these, so they're harder.
- **Kerbs:** long single steps.
- **Sunken stairs:** a plateau with steps down into a pit and back up.
- **Uneven flights:** half of all staircases have steps that each vary ±35% in height.

Risers go up to 5 cm at full level, on treads of 24–45 cm. Raising the risers to 6 cm stalled progress for five rounds, so they went back to 5 cm. In 30% of training episodes (**stair practice**) a straight flight is put right in his path and he's told to walk forward at 0.15–0.3 m/s, so he practises climbing instead of meeting stairs by chance.

`keep_best.py` tests him on eight staircases (`STAIR_CASES`), two tries each: pyramids of 3, 4 and 5 cm; straight 4 cm flights with deep (40 cm) and tight (25 cm) treads; an uneven flight (3/5/3.5 cm); a flight approached at 30°; and a 5 cm kerb. The 128M walker gets over the 3 and 4 cm pyramids every time and the kerb half the time, but none of the straight flights: he climbs the first step, stalls at the second, slides along the riser and walks off the side.

The walker is blind, so it has to learn to climb by feel. Two things help:

- **Sharper risers:** the height field has 2.5 cm cells (it was 5 cm, which turned every riser into a 5 cm ramp).
- **A stumble penalty:** a planted foot pressed against a riser costs points, so he learns to lift his feet higher. "Pressed against a riser" means ground more than 2 cm higher right beside the foot, judged from the terrain itself. MuJoCo's sphere-on-heightfield contacts have tilted normals even on flat ground, so the usual contact-force test (legged_gym's `feet_stumble`) fired 86 times in 500 steps on flat floor. `keep_best.py` tests it on a staircase of 3 steps up and 3 down at 4, 6 and 8 cm risers. The walker from before stairs training (walk_turn2 79M) gets over none of them: with 4 cm risers it climbs the first step and stalls.

The Webots world has two staircases (6 cm and 4.5 cm risers) and an 8 cm platform. They sit below the lidar's scan plane, so the navigator treats them as walkable floor.

## Watch it

```bash
.venv/bin/python play.py runs/walk --video walk.mp4 --cmd 0.2 0 0   # vx, vy (m/s), yaw rate (rad/s)
.venv/bin/mjpython play.py runs/walk --viewer                       # live 3D window
```

## Demos in Webots

```bash
scripts/demo.sh             # all four, about 6 minutes: tour, stairs, flip, walls
scripts/demo.sh stairs      # or just one
```

`scripts/demo.sh` starts the sim in planner mode if it isn't already, and `rl/demo.py` runs the demos, narrating what the navigator is doing. The Webots camera follows Bob.

**From the Webots window:** with the sim in planner mode (`scripts/start_sim.sh --nav planner`), click the 3D view, then press **1** tour, **2** stairs, **3** flip, **4** walls, **0** all, or **S** to stop. The bridge runs `demo.py` itself. A key guide and Bob's current status are drawn at the top left of the view.

| demo | what happens | last run |
| --- | --- | --- |
| tour | six targets round the arena, past the boxes and walls | 4 of 6 within 75 s (now 100 s each) |
| stairs | spots the target on the 3 cm staircase's landing, climbs up to it | reached in 29 s, body 8 cm higher |
| flip | flipped upside down, rights himself, walks to a target | up in 10 s, target in 10 s more |
| walls | target hidden behind the south wall: looks, searches, spots it, goes round | reached in 77 s |

### Do MuJoCo and Webots match?

They run the same files (`policies/*.npz`, `nav_spec.py`, `spec.py`), checked by hash inside the sim container. The walker, measured with the same commands (MuJoCo: `speed_eval.py` and `turn_eval.py`; Webots: `webots_eval.py` in the open arena):

| command | MuJoCo | Webots |
| --- | --- | --- |
| walk 0.15 m/s | 0.247 | 0.266 |
| run 0.30 m/s | 0.317 | 0.243 |
| turn 0.4 rad/s | 0.46 | 0.49 |
| fast turn 1.0 rad/s | 0.89 | 0.91 |

Walking and turning match closely. Running is ~23% slower in Webots, a gap in how the two simulators handle foot contact. The planner's dead reckoning uses the MuJoCo speeds, so in Webots it thinks he's gone a bit further than he has. That's why it only treats a remembered target as reached within 0.15 m, and otherwise keeps homing in until it re-sights it.

## Run it in Webots

To watch a training run live, use `scripts/start_sim.sh --nav scripted --follow-training rl/runs/<name>` while `keep_best.py runs/<name>` runs beside `train.py`. The watcher scores every 1M-step snapshot (turning, speed, step quality and drift), keeps the best in `best.npz` and logs every score to `best.md`. The bridge swaps in each new best within 5 s of sim time.

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
5. **Obstacles (lidar):** if a wall or box is within 55 cm ahead while walking, turn in place towards the more open side until the way ahead is clear past 85 cm, walk on for 1.5 s, then head for the target again. It also eases away from anything within 35 cm of either side. The target post is not counted as an obstacle.

Turns go up to 1.0 rad/s, slowing as he lines up, and he searches at 0.8 rad/s. When the goal moves (`/goal`), the Webots bridge holds him still for 0.4 s while the camera takes a first look. Without that he span off at search speed and then turned back, which looked like turning the wrong way. Walls also hide the target from the camera model, judged from the lidar.

**Among obstacles** (`nav_eval.py baseline --obstacles`, 20 one-minute runs, random walls and boxes): the lidar rule brings bumps down from 7.05/min to 0.45/min, and goals up from 4.2/min to 4.5/min. In Webots he keeps 38–44 cm from walls. He can still fail to find a target hidden behind a wall, because the scripted navigator keeps no map; the trained lidar navigator below is meant to fix that.

**Lidar navigator** (`nav_train.py --obstacles --imitate 300000`, runs/nav_lidar): trained among random walls and boxes, with lidar in its observation. Bumping costs a lot: 40 points for each new bump (about four goals' worth), 20/s while touching, a little for getting within 45 cm, and 3/s for being stuck (2 s without moving or turning). It starts by imitating the scripted navigator, then PPO improves on it. `nav_eval.py <npz> --obstacles` scores it.

Results over 20 one-minute MuJoCo runs among random obstacles:

| | goals/min | bumps/min | stuck |
| --- | --- | --- | --- |
| scripted + lidar | 5.0 | 0.20 | 0% |
| lidar navigator (nav_lidar 3.25M, installed as `policies/rocky_nav_lidar.npz`) | 6.8 | 0.00 | 1% |
| the same, as the hybrid Webots runs (`--hybrid`) | 5.9 | 0.05 | 2% |

The trained navigator never learned to explore: in training a target nearly always came into view within a turn, so when one stays hidden (behind a wall) it just spins. `nav_spec.HybridNavigator` therefore lets it drive only while the target is in sight or was seen in the last 1.5 s. Otherwise the scripted search takes over: turn once, then walk off round obstacles and look again. The Webots bridge runs lidar navigators this way (`--nav rl/policies/rocky_nav_lidar.npz`), and `train_rounds.py` scores them this way too.

Both navigators also have a **stuck escape** (`nav_spec.StuckEscape`). If he walks at a target he can see for 2.4 s and gets less than 8 cm closer (pushing against a low step the lidar can't see, say), he turns away about 75° and walks on for 1.5 s before trying again.

**The planner** (`nav_spec.PlannerNavigator`, `--nav planner`) is the navigator to use with the lidar. It's hand-written, not trained:

- **Memory:** every sighting of the target goes into an odometry frame (the gyro for heading, the walker's measured speed for distance, since a 0.15 m/s command actually walks at 0.205). A target hidden by a wall is still known, and its bearing stays right as he moves.
- **Free-path steering:** of 72 headings, it takes the free one nearest the target, where his whole footprint fits along it (the legs reach ~0.35 m from the centre). That leads round the end of a wall instead of into it. He turns in place first if that heading is far off, then walks or runs with gentle steering.
- **Search:** with no target known, he looks all round, then walks off towards open space he hasn't visited, preferring wall ends with unseen space behind them, and looks again. (Steering the search by what the camera has already covered measured worse, so it's off: `SEARCH_COVERAGE`.)
- **Steps and stairs:** the lidar scans ~36 cm up, so it can't see steps. If the scan stops changing while he's walking, he's stuck. If the way ahead looks clear, it's probably a step, so he first tries to climb it slowly for 4 s. If that fails he remembers it as an invisible wall for 2 minutes and goes round.
- **Safety layer** (`nav_spec.safe_command`, used by every navigator): each command is checked against the lidar by predicting his footprint 0.6 s ahead, including the sweep of a turn. A command that would come within 8 cm of anything is swapped for the closest safe one. If he's already too close, it picks whichever move (including stepping sideways) gets him out fastest, because standing still isn't safe either: the walker drifts. The target post is masked out only at its own distance, so a wall right behind a goal still counts.

Results over 20 one-minute MuJoCo runs among random walls and boxes (`nav_eval.py <nav> --obstacles --goals random|near|hidden`):

| goals/min (bumps/min) | random | near a wall (0.3-0.5 m) | hidden behind a wall |
| --- | --- | --- | --- |
| scripted | 4.8 (0.00) | 1.9 (0.20) | 0.5 (3.20) |
| hybrid trained | 4.2 (0.00) | 1.2 (1.55) | 0.3 (0.05) |
| **planner** | **4.8 (0.00)** | **1.8 (0.00)** | **0.9 (0.00)** |

In Webots (`rl/wall_test.py`: each goal from the arena centre, 2 minutes each), the planner reaches 3 to 4 of the 6 goals. It's reliable on open floor and beside the boxes (about 20 s each). It's hit-and-miss for goals hidden behind the long walls, because without a map the search is guesswork; he once wandered south while the target was just round the north end of a wall. Nearest approaches in Webots were mostly 0.3 m or more, with two rare runs at 0.14 and 0.19 m.

**Stairs in Webots** (`rl/stairs_test.py east|north`): with the 92M walker he walks up to the 3 cm east staircase, notices the first step, climbs to the landing and reaches the target there (2 of 3 tries, 30–48 s; the third came at the corner and stalled). The 4.5 cm north staircase is still too high: he stalls at the first riser, tries to climb, and keeps retrying. In MuJoCo he crosses 3–4 cm stairs most of the time but not 5 cm.

**Footprint:** the safety layer models him as a 0.62 m square with its corners rounded off at 0.38 m, because his feet sit at the diagonals and reach only ~0.35 m. A plain square swung its corners out to 0.44 m when turning, so in tight spots it refused safe turns and backed up instead (and he barely walks backwards). A plain circle bumped occasionally. It also predicts with the walker's real speeds, which are about 20% of the command going backwards and 80% sideways.

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
- **Outputs:** 12 joint-angle offsets from the standing pose (hip 0.75 rad, knee 1.05 rad), sent at 50 Hz. One unit of action reaches the servo limit (less a 10% guard against grinding the gears into their end stops), so the only limits are the servos and the robot's shape. The legs collide with the body and each other, in MuJoCo and in Webots (`selfCollision TRUE`). Walkers used to be trained on narrow bands around standing (yaw 0.35–0.6, hip 0.45, knee 0.28 rad per unit), which capped foot lift at 4.6 cm, too little for 4.5 cm stairs; the full range allows about 7 cm. `rescale_actions.py` converts a checkpoint or `.npz` between ranges without changing what it does, and every walker in `policies/` has been converted.
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
  - a reward for reaching the commanded turn rate (`turn_progress`, linear like `run_progress`), with turn commands up to 1.0 rad/s
  - wide hip-yaw swings (±0.6 rad, `spec.ACTION_SCALE`) and a 0.8 s stride (`spec.TURN_PERIOD`), so each step turns the body further

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

## Lidar

Rocky-Bob carries a Slamtec RPLIDAR C1 on his lid (`cad/rocky_bob/README.md`). It isn't used by any policy yet. It is there so a navigator can learn to avoid obstacles, and so real rooms can be scanned and rebuilt in simulation. All three places produce the same scan:

| where | source | message |
| --- | --- | --- |
| real robot | `sllidar_ros2` (`ros2 launch sllidar_ros2 sllidar_c1_launch.py`) | `/scan`, 10 Hz, ~500 points |
| Webots | the bridge, from the robot's `Lidar` device | `/scan`, frame `laser`, ~10 Hz, 500 points |
| MuJoCo | `lidar.LidarModel` (`mj_multiRay` from the `lidar` site, 0.06 ms a scan) | 72 sectors, refreshed at 10 Hz |

**See the point cloud:** `scripts/start_sim.sh` also starts `scan_viewer.py`, so open **http://localhost:8765**. It shows the live scan round the robot, coloured by distance. With a pose available (Webots publishes `/sim/position` and `/imu`), every return also goes into a 5 cm grid, so walking around builds a map of the room. Scroll to zoom, drag to pan, and use *clear map* to start over. In Webots itself, *View > Optional Rendering > Show Lidar Point Cloud* draws the rays in 3D. On the Pi, run `python3 rl/scan_viewer.py` next to `sllidar_ros2` and browse to `http://<pi>:8765`. The real robot has no odometry yet, so the viewer stays in the robot's frame there, with no map.

For learning, a scan is reduced to 72 five-degree sectors (`lidar.scan_bins` for a ROS scan). Each sector keeps the nearest return, clipped to 4 m, counter-clockwise from straight behind. The MuJoCo model adds 1.5 cm noise and 2% dropped sectors.

A 2D lidar only sees what crosses its scan plane, about 36 cm off the floor. In the Webots obstacle world the walls and two boxes are tall enough to show up, while the small box, the rocks and the steps are not. The real robot has the same blind spot.

**Real robot:** plug the C1 into its USB adapter on the Pi, or wire it to the Pi's UART at 460800 baud. Install `sllidar_ros2` and run its C1 launch file. A Webots scan was checked against a ray-cast of the world, and 66 of 72 sectors agreed to within 10 cm.

## Floor sensor (stairs)

The lidar only sees a slice about 36 cm up, so steps, kerbs and ledges are invisible to it. A **SparkFun Qwiic ToF Imager, VL53L5CX** ([SEN-18642 on Amazon](https://www.amazon.com/SparkFun-VL53L5CX-Multizone-Measurements-Recognition-Laser-Assisted/dp/B09M97318D)) in the nose looks at the floor instead. It reads an 8 × 8 grid of distances over about 45° × 45°, up to 4 m, at 15 Hz. It sits behind a small window in a chamfer under the mustache, tilted 24° down, covering the floor from about 0.3 m to 3 m ahead (`cad/rocky_bob/body.py`: `TOF_*`).

**Wiring:** it runs at 3.3 V on the Pi 5's I²C (address 0x29), through a Qwiic-to-Pi cable (e.g. SparkFun PRT-14425). Pimoroni's `vl53l5cx-python` works with it.

`tof.py` turns the 64 distances into the floor height where each zone lands, relative to the floor he's standing on, using the IMU's gravity to undo body tilt. Flat floor reads ~0, a 4 cm step +0.04, a drop −0.04. MuJoCo casts the 64 rays from a `tof` site. Webots has an 8 × 8 `RangeFinder` at the same spot. Webots reports depth along the sensor's axis, so `tof.from_depth` converts it to true distances. In Webots the bridge publishes `/tof/heights`.

- **Walker:** a walker can take the 64 heights as extra inputs: 99 instead of 35. Older walkers ignore them; `policy.py` uses only the inputs a policy has. `widen_obs.py` gave the champion the new inputs with zero weights, so it starts out behaving the same. After 1M steps of training (129M) it already crossed 44% of the test staircases instead of 12%, with fewer taps (20 against 28 per run), half the backing up (4.4 s against 7.9 s), and a higher top speed (0.365 against 0.319 m/s). But it turned and tracked straight lines worse. A learning rate of 1e-4 then made it collapse, so training continues from 129M at 3e-5.
- **Planner:** it judges each step by the jump between neighbouring zones, not the total height. A staircase is many small climbable jumps even if its top is 10 cm up. A single jump over 5 cm (a wall, ledge or kerb face) or a drop over 8 cm becomes an obstacle, like a lidar return, so he plans round it before reaching it. A climbable step of 2.5 cm or more, seen in at least two zones, slows him to a walk taking it straight. The target's own post and the bottom of walls aren't counted as steps.

**Results so far:**

- **MuJoCo, flat floor:** the planner with the sensor is as good as without it (5.4 goals/min, no bumps).
- **MuJoCo, rough training terrain:** it's slower (about 3.6 against 4.8 goals/min). It slows down at steps that the current walker, which can't see the floor, often fails to climb anyway.
- **Webots:** he now sees the ledge, platform, rocks and small box, which the lidar misses, and doesn't walk into them. A target tucked into the 30 cm gap between the ledge and the east staircase is still out of reach: getting there means climbing onto the stairs and walking along them, and the planner doesn't plan routes across steps yet.

## Moving to the real robot

- **Add an IMU.** The policy needs gravity direction and gyro rates. A BNO085 or MPU-6050 on the Pi's I2C bus works. Publish it on `/imu`.
- **Run open loop.** MG995s can't report their angle, so run with `python3 rl/ros_policy_node.py rl/runs/walk/policy.npz --open-loop`. The node then feeds its own last commands back in place of measured angles. Set the servo bridge's joint directions and offsets first, using `/jdirection` and `/joint_offset`.
- **Expect a gap.** Real servos have backlash, delay and droop. Start with the robot held up off the ground, use small commands, and stay within reach of the power switch.

## Limits

- **Standing up:** the robot can't push itself up from lying on its belly, because the hip servos stall. Training therefore always starts from standing, and Webots spawns it standing.
- **Weight:** standing takes about 21% of the MG995's stall torque. Walking on rough ground takes more, and the torque-limit penalty keeps it below 75%. Lighter prints or stronger servos give the gait more headroom.
