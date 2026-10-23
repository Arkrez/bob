# Trained walking policies

Numpy-only (`policy.py`), for `ros_policy_node.py`, `play.py` and `watch.py`. Trained with `walk_steady` (fine-tuned from `walk_v3` at 25M steps).

| file | snapshot | notes |
| --- | --- | --- |
| `rocky_walk.npz` | walk_hp4 42M | Default. Trained for the HobbyPark 25 kg servos: clean, even ~4 cm steps, almost no bounces, fewest steps when pivoting. |
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

## HobbyPark 25 kg servos: `rocky_walk.npz` (walk_hp4 42M)

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
