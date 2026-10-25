# Trained walking policies

Numpy-only (`policy.py`), for `ros_policy_node.py`, `play.py` and `watch.py`. All walkers here use the 0.6 rad hip-yaw action scale (`spec.ACTION_SCALE`). The older ones were trained at 0.35 and converted with `rescale_actions.py`, which leaves them behaving as before. Trained with `walk_steady` (fine-tuned from `walk_v3` at 25M steps).

| file | snapshot | notes |
| --- | --- | --- |
| `rocky_walk.npz` | round_w0030 178M | Default. Floor sensor, full servo range. Gets over 88% of the stair test (3-5 cm, 17 s each, 5.6 taps/run), walks 0.30 m/s (top 0.36), no drift. Turns a bit slower (0.76 rad/s) and slides ~12 cm turning on the spot. Not yet over 7-10 cm steps or ledges. |
| `rocky_walk_128M.npz` | round_w0006 128M | Previous default: blind (35 inputs), climbs 3-4 cm pyramids in MuJoCo by feel. |
| `rocky_walk_79M.npz` | walk_turn2 79M | Default. Best of a further 30M-step fine-tune, picked by `keep_best.py`: turns at 0.85 rad/s in 7.2 footfalls/rad, runs at 0.32 m/s, barely bounces, and drifts 9 cm (not 35) over a 6 s straight walk. |
| `rocky_walk_turn48.npz` | walk_turn 48M | First fast-turn walker: 0.73 rad/s, 8.1 footfalls/rad. |
| `rocky_walk_hp4.npz` | walk_hp4 42M | Previous default: slower turns, but the cleanest, most even ~4 cm steps. |
| `rocky_walk_pivot_mg995.npz` | walk_pivot 30M | Previous default, trained for MG995s. Holds its pivot best (4.6 cm drift), but takes low, uneven steps. |
| `rocky_walk_run22.npz` | walk_run 22M | Faster walking and running, but sloppier turns in place. Needs `spec.TURN_PERIOD = 0.6`. |
| `rocky_walk_v1.npz` | walk_steady 45M | The previous walker. Can walk backward and sidestep, but can't run. |
| `rocky_walk_turny.npz` | walk_steady 35M | Older walker that turns in place better. |
| `rocky_nav.npz` | nav_v2 | Turn-then-walk navigator for `rocky_walk.npz`; see below. |
| `rocky_nav_v1.npz` | nav 2.00M | Previous navigator, for `rocky_walk_v1.npz`. It turns while walking. |

Both were tested on 8 seeds with randomised physics, medium terrain and shoves every 2–5 s. Neither fell in any of the 40 runs per policy. Speeds are commanded vs achieved: mean, with the worst seed in brackets.

| policy | forward 0.15 m/s | back −0.10 | turn in place 0.40 rad/s | sidestep 0.08 | roughest terrain 0.15 | servo torque, 95th pct |
| --- | --- | --- | --- | --- | --- | --- |
| rocky_walk (45M) | 0.158 (0.115) | −0.050 | 0.295 | 0.058 | 0.109 | 0.74 N·m |
| rocky_walk_turny (35M) | 0.154 (0.128) | −0.050 | 0.404 | 0.025 | 0.106 | 0.77 N·m |

`rocky_walk.mp4` shows the 45M policy walking forward, turning, sidestepping and crossing rough ground.

Weak spots:
- **Walking backward:** it reaches about half the commanded speed.
- **Rough ground:** on the roughest terrain a few seeds get stuck on blocks.
- **Torque:** it still reaches about 80% of stall in bursts.

In Webots, with the bridge running it in lockstep with sim time, it walks forward at 0.135 m/s, backward at −0.057, turns in place at 0.29 rad/s and sidesteps at 0.055 m/s; see `../README.md`. To run it, from the repo root:

```bash
scripts/start_sim.sh --policy rl/policies/rocky_walk.npz
```

## Navigators

The default is **`--nav scripted`**: spin to find, turn to face, then walk or run. See `../README.md`. These numbers are with the walk+run walker, from `nav_eval.py`: 20 one-minute runs on medium terrain with shoves and the Pi camera model.

| | targets/min | time to target, median | path efficiency | arcing | falls |
| --- | --- | --- | --- | --- | --- |
| **scripted (default)** | **4.1** | 12.5 s | **94%** | 5% | 0/20 |
| `rocky_nav.npz` (nav_v3 3.25M, trained turn-then-walk) | 3.1 | 12.1 s | 85% | 5% | 0/20 |
| `rocky_nav_v1.npz` on `rocky_walk_v1`, for reference | 4.8 | 10.8 s | 86% | 42% | 0/20 |

`rocky_nav_v1.npz` was trained when `nav_spec.CMD_HIGH` topped out at 0.18 m/s. To use it, set `CMD_HIGH` back to `[0.18, 0.08, 0.5]` and use `rocky_walk_v1.npz`.

## Walk and run: `rocky_walk.npz` (walk_run, 22M)

Trained from scratch with running commands up to 0.35 m/s. The gait clock quickens from a 0.6 s stride to 0.4 s as commanded speed rises above 0.18 m/s (`spec.gait_period`), so running needs no extra input: it's just a faster `/cmd_vel`. The stable-body ("chicken head") bonuses were included from the start.

Tested on 6 seeds per command with randomised physics, medium terrain and shoves. Neither walker fell in any of its 36 runs. Steady and smooth ride are the stable-body scores, where 1 means a perfectly still body.

| | walk 0.15 | run 0.30 | turn 0.40 | back −0.10 | sidestep 0.08 | rough 0.15 | steady | smooth ride | torque, 95th pct |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `rocky_walk_v1` | 0.142 | 0.104 | 0.289 | −0.054 | 0.056 | 0.111 | 0.45 | 0.20 | 0.80 N·m |
| **`rocky_walk` (22M)** | 0.144 | **0.254** | **0.344** | −0.003 | 0.014 | **0.117** | **0.67** | **0.24** | **0.71 N·m** |

The new walker gave up walking backward and sidestepping in exchange for running. The turn-then-walk navigator doesn't need either. If you want them back, give the backward and sidestep commands a bigger share in `env.COMMAND_MIX` and fine-tune.

## Pivot turns: `rocky_walk.npz` (walk_pivot 30M)

This walker is fine-tuned from `walk_run` with rewards for turning in place:

- a bonus for keeping the body centre over the point where the turn began
- a small cost per footfall
- a cost for the body sliding while it turns

It also uses a slower 1.0 s stride when turning in place (`spec.TURN_PERIOD`), so it takes fewer, bigger steps.

Turning results are from `turn_eval.py`: 10 s turns, both directions, randomised physics. Walking results are as above.

| | footfalls per radian | drift off the pivot | turn rate (cmd 0.4) | walk 0.15 | run 0.30 | rough 0.15 | targets/min (scripted nav) | path efficiency |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `rocky_walk_run22` | 23.7 | 11.8 cm | 0.34 | **0.144** | **0.254** | **0.117** | **4.1** | **94%** |
| **`rocky_walk` (pivot 30M)** | **20.5** | **7.2 cm** | 0.31 | 0.121 | 0.225 | 0.096 | 4.0 | 82% |

**To switch back to the faster walker:** set `TURN_PERIOD = 0.6` in `spec.py` and copy `rocky_walk_run22.npz` over `rocky_walk.npz`.

## Longer fine-tune: `rocky_walk.npz` (walk_turn2 79M)

This run continued from walk_turn's 49M checkpoint at half the learning rate (`5e-5`) for 30M steps. `keep_best.py` scored every 1M-step snapshot and kept the best; the whole table is in `runs/walk_turn2/best.md`. The score covers turning, footfalls, walking and top speed, step height, pivot bounces, walking drift and turn slide. Snapshots that fell, or lost more than 10% of walking or top speed, were rejected.

| | turn rad/s (cmd 1.0) | footfalls/rad | walk 0.15 | top speed | step height | pivot bounces/s | walk drift (6 s) | turn slide |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `rocky_walk_turn48` | 0.73 | 8.1 | 0.196 | 0.284 | 3.4 cm | 0.8 | 35 cm | 4 cm |
| **`rocky_walk` (79M)** | **0.85** | **7.2** | **0.208** | **0.317** | **3.8 cm** | **0.1** | **9 cm** | **1.2 cm** |

## Fast turns: `rocky_walk_turn48.npz` (walk_turn 48M)

This walker is fine-tuned from walk_hp4 (44M checkpoint) to turn faster with fewer steps:
- Hip-yaw range is 0.6 rad per unit action, up from 0.35. Turning in place is limited by how far each step can swing the hip: a planted foot is further from the body's centre than the hip, so the hip sweeps about 1.7× the body's rotation. The checkpoint was converted with `rescale_actions.py` first, so training started from identical behaviour.
- The turn-in-place stride is 0.8 s, down from 1.0 (`spec.TURN_PERIOD`).
- Turn commands go up to 1.0 rad/s, and a linear `turn_progress` reward pays for the share of the commanded rate reached.

Trained gently (`--lr 1e-4 --target-kl 0.01`, full penalties) for 12M steps. The snapshot 4M steps in was the best. Later ones turned slower and lost running speed.

Turning is from `turn_eval.py --wz 0.4 1.0`, both walkers under the current spec:

| | turn rate (cmd 1.0) | footfalls/rad (cmd 1.0) | pivot drift (cmd 1.0) | turn rate (cmd 0.4) | walk 0.15 | top speed | step height (walk) | bounces/s (pivot) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `rocky_walk_hp4` | 0.38 rad/s | 14.9 | 20.3 cm | 0.31 | 0.214 | 0.282 | **4.0 cm** | **0.0** |
| **`rocky_walk` (turn 48M)** | **0.72 rad/s** | **8.4** | **5.2 cm** | **0.41** | 0.198 | **0.283** | 3.4 cm | 1.2 |

The navigator now turns at up to 1.0 rad/s, slowing as it lines up (`nav_spec.FACE_TURN`), and searches at 0.8 rad/s. Over 20 one-minute `nav_eval.py baseline` runs it reaches 4.8 goals/min (worst run 4), up from 4.4 (worst run 3), with path efficiency 85% against 92%.

**Webots** (open arena, `rl/webots_eval.py`): walk 0.205 m/s, run 0.265, turn 0.39 rad/s at cmd 0.4 and **0.86 rad/s at cmd 1.0**, sidestep 0.065, body wobble 0.3–0.5 rad/s, no falls.

## HobbyPark 25 kg servos: `rocky_walk_hp4.npz` (walk_hp4 42M)

This walker was trained from scratch for the HobbyPark 25 kg coreless servo, modelled at 7.4 V as 2.45 N·m stall and about 8 rad/s (`spec.py`). It runs with commands up to 0.8 m/s.

On top of the earlier rewards, it adds:
- a target step height (4.5 cm) that every leg should reach
- a penalty when the four legs' step heights differ
- a bounce penalty (a re-touch after less than 0.08 s in the air)
- stricter pivot rewards: an anchor bonus and a linear offset penalty, a slide penalty, and no foot planted longer than 0.75 s

Everything below is measured with the new servo model. The step test's target height is 4.5 cm.

| | walk 0.15 | run 0.30 | top speed | turn 0.40 | rough 0.15 | pivot drift | footfalls/rad | step height (walk) | bounces/s (walk, pivot) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `rocky_walk_pivot_mg995` | 0.125 | 0.189 | **0.314** | 0.28 | 0.114 | **4.6 cm** | 26.7 | 1.7 ± 0.8 cm | 1.7, 0.8 |
| **`rocky_walk` (hp4 42M)** | **0.188** | **0.247** | 0.280 | **0.34** | **0.168** | 10.8 cm | **14.4** | **4.0 ± 0.5 cm** | **0.0, 0.2** |

**Webots** (open arena, `rl/webots_eval.py`, with the servo rotor inertia added to the Webots legs):

| | walk 0.15 m/s | run 0.30 m/s | turn in place 0.40 rad/s | sidestep 0.08 m/s | body wobble |
| --- | --- | --- | --- | --- | --- |
| Webots | 0.198 | 0.240 | 0.36 | 0.056 | 0.3–0.5 rad/s |
| MuJoCo | 0.197 | 0.256 | 0.34 | | 0.4–0.5 rad/s |

Before the rotor inertia was added, Webots managed only about half this speed with 2.0–2.3 rad/s of wobble.

**Not yet achieved:**
- Pivot drift is still about 10 cm; the pivot rewards struggled to hold the body over the spot.
- Top speed stays around 0.3 m/s even with the stronger servo, so the limit now looks like the learned gait, not the actuator.
