---
description: Start the Bob simulation (Webots on the Mac + ROS 2 in Docker)
argument-hint: "[rocky|cad] [--demo] [--rebuild]"
allowed-tools: Bash(scripts/start_sim.sh:*), Bash(scripts/sim_ros.sh:*), Bash(docker logs:*), Bash(docker compose:*)
---

Start Bob's simulation from the repo root.

1. Build the option list from `$ARGUMENTS`:
   - A bare `rocky` or `cad` becomes `--world rocky` or `--world cad`.
   - Pass any other flags through as they are.
   - Always add `--detach`.

   Then run `scripts/start_sim.sh <options>`. The first run builds the Docker image and the ROS workspace, which can take 10 minutes or more, so run it in the background and wait for it to finish rather than polling.
2. When it finishes, check the result:
   - Run `scripts/sim_ros.sh ros2 topic echo /joint_states --once` and confirm it reports 12 joint positions.
   - Run `scripts/sim_ros.sh ros2 topic info /leg_position_controller/commands` and confirm the topic has a subscriber.
3. If the bridge did not connect, read `docker logs --tail 80 bob-sim` and `${TMPDIR:-/tmp}/bob-sim/webots.log`. Explain what failed. Common causes:
   - Docker isn't running.
   - The Webots version isn't R2025a.
   - The simulation is paused.
   - Another Webots instance is already holding port 1234.
4. Report in a few lines:
   - Which world is running.
   - Whether the 12 motors are live.
   - How to stop it: `scripts/start_sim.sh --stop`.

Never enable the physical servo bridge from this command.
