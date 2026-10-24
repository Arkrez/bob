"""Self-righting as Bob runs it: the learned roll-over, then a scripted tidy-up into the standing pose.

Shared by recover_eval.py and the Webots bridge (numpy only, like spec.py).

The learned policy (recover_env.py) rolls him upright from his back or side in about a second,
but ends in a stable splayed stance (hip-yaw joints at their limit) that the walker can't start
from. Training couldn't pull him out of it: swinging a loaded leg's yaw drags its foot. So once
he's upright and steady, the tidy-up moves one leg at a time while the other three carry him:
lift the foot, swing the yaw home, put it down at the standing pose. If he tips over again on the
way, it goes back to rolling.
"""

import numpy as np

from spec import CONTROL_DT, STAND, TARGET_LIMIT, observation, recovery_targets, slew

UPRIGHT = 0.95          # up-vector z
STEADY_RATE = 0.8       # rad/s: body rotation below this counts as steady
STEADY_TIME = 0.3       # s upright and steady before tidying up
LIFT = (-0.5, 0.3)      # hip and knee offsets from STAND while a foot is lifted
LEG_ORDER = (0, 2, 1, 3)
MOVE_TIME = 0.35        # s for each of lift / swing / place (0.25 was too hasty: 62% vs 83%)
SETTLE_TIME = 0.6       # s holding STAND at the end


class Recovery:
    def __init__(self, policy):
        self.policy = policy
        self.reset(STAND)

    def reset(self, q):
        self.mode, self.steady_for = "roll", 0.0
        self.action = np.clip(np.asarray(q, dtype=float) / TARGET_LIMIT, -1, 1)
        self.targets = np.array(q, dtype=float)
        self.plan = []

    def done(self):
        return self.mode == "done"

    def _tidy_plan(self, q):
        """Joint-target keyframes (targets, seconds): one leg at a time, then settle."""
        target, plan = np.array(q, dtype=float), []
        for leg in LEG_ORDER:
            j = slice(3 * leg, 3 * leg + 3)
            lifted = target.copy()
            lifted[j] = [target[3 * leg], STAND[1] + LIFT[0], STAND[2] + LIFT[1]]
            plan.append((lifted.copy(), MOVE_TIME))
            lifted[3 * leg] = STAND[0]
            plan.append((lifted.copy(), MOVE_TIME))
            target[j] = STAND[j]
            plan.append((target.copy(), MOVE_TIME))
        plan.append((STAND.copy(), SETTLE_TIME))
        return plan

    def step(self, gravity, gyro, q):
        """One control step (spec.CONTROL_DT): returns the 12 joint targets."""
        up = -gravity[2]
        if self.mode in ("tidy", "done") and up < 0.5:      # tipped again: roll again
            self.reset(q)
        if self.mode == "roll":
            steady = up > UPRIGHT and float(np.linalg.norm(gyro)) < STEADY_RATE
            self.steady_for = self.steady_for + CONTROL_DT if steady else 0.0
            if self.steady_for >= STEADY_TIME:
                self.mode, self.plan, self.left = "tidy", self._tidy_plan(q), None
            else:
                obs = observation(gravity, gyro, np.zeros(3), q, self.action, 0.0)
                self.action = np.clip(self.policy(obs), -1, 1)
                self.targets = slew(self.targets, recovery_targets(self.action))
                return self.targets
        if self.mode == "tidy":
            if self.left is None:
                self.left = self.plan[0][1]
            self.left -= CONTROL_DT
            target = self.plan[0][0]
            self.targets = slew(self.targets, target)
            target = self.targets
            if self.left <= 0:
                self.plan.pop(0)
                self.left = None
                if not self.plan:
                    self.mode = "done"
            return target
        return STAND.copy()
