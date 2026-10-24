"""Webots demos of what Bob can do. Run with the sim in planner mode:

    scripts/start_sim.sh --nav planner
    scripts/sim_ros.sh python3 /workspace/rl/demo.py            all of them, in order (~6 min)
    scripts/sim_ros.sh python3 /workspace/rl/demo.py tour       or one: tour, stairs, flip, walls

  tour    walk to a run of targets round the arena, steering clear of the walls and boxes
  stairs  see the target on the landing of the east staircase, walk up it and reach it
  flip    turned upside down, right himself, then walk on to a target
  walls   a target hidden behind a wall: look, search, spot it and go round the wall to it

Or from the Webots window (planner mode): click the 3D view and press 1 tour, 2 stairs, 3 flip,
4 walls, 0 all, S stop; the bridge starts this script itself.

Each demo puts him in place first (/place), moves the orange target (/goal) and narrates what
the navigator says it's doing (/nav/status). The Webots camera follows him.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String

TITLES = {
    "tour": "walk to a run of targets round the arena, steering clear of the walls and boxes",
    "stairs": "see the target on the landing of the east staircase, walk up it and reach it",
    "flip": "turned upside down, right himself, then walk on to a target",
    "walls": "a target hidden behind a wall: look, search, spot it and go round the wall to it",
}
DEMOS = {
    "tour": dict(start=(0.0, 0.0, 0.0), goals=[(1.0, 0.5), (2.25, 1.8), (1.0, -0.6), (-0.5, -1.2), (-2.0, -1.2), (0.0, 0.0)]),
    "stairs": dict(start=(3.4, 0.55, -math.pi / 2), goals=[(3.35, -1.05)]),
    "flip": dict(start=(0.0, -0.5, 0.0), flip=180.0, goals=[(1.0, 0.5)]),
    "walls": dict(start=(0.0, 0.0, 0.0), goals=[(-1.6, -2.8)], timeout=150.0),
}


class Demo:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("bob_demo")
        self.status, self.pos = "", (0.0, 0.0, 0.0)
        self.n.create_subscription(String, "/nav/status", self._status, 5)
        self.n.create_subscription(PointStamped, "/sim/position",
                                   lambda m: setattr(self, "pos", (m.point.x, m.point.y, m.point.z)), 5)
        self.place_pub = self.n.create_publisher(PointStamped, "/place", 5)
        self.goal_pub = self.n.create_publisher(PointStamped, "/goal", 5)
        self.flip_pub = self.n.create_publisher(Float64, "/flip", 5)
        self.last_said = ""
        self.spin(1.5)

    def _status(self, msg):
        self.status = msg.data

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def say(self, text):
        print(f"  {text}", flush=True)

    def point(self, x, y, z=0.0):
        p = PointStamped()
        p.header.frame_id = "world"
        p.point.x, p.point.y, p.point.z = float(x), float(y), float(z)
        return p

    def narrate(self):
        what = self.status.split(";")[0]
        if what and what != self.last_said and not what.startswith("tracking target"):
            self.say(f"... {what}")
        elif what.startswith("tracking target") and not self.last_said.startswith("tracking target"):
            self.say("... got the target in sight, heading for it")
        self.last_said = what

    def run(self, name):
        d = DEMOS[name]
        print(f"\n== {name}: {TITLES[name]}", flush=True)
        for _ in range(2):   # the first message can go out before the bridge is listening
            self.place_pub.publish(self.point(*d["start"]))
            self.spin(1.0)
        self.spin(1.5)
        if "flip" in d:
            self.say(f"flipping him {d['flip']:.0f} degrees")
            self.flip_pub.publish(Float64(data=d["flip"]))
            self.spin(1.5)
            t0 = time.time()
            while time.time() - t0 < 30 and "righting" not in self.status:
                self.spin(0.2)
            self.say("... righting himself")
            while time.time() - t0 < 30 and "righting" in self.status:
                self.spin(0.2)
            self.say(f"back on his feet after {time.time() - t0:.0f} s")
        reached = 0
        for gx, gy in d["goals"]:
            self.say(f"target moved to ({gx:+.1f}, {gy:+.1f})")
            self.goal_pub.publish(self.point(gx, gy))
            self.spin(1.0)
            self.status, self.last_said = "", ""
            t0 = time.time()
            while time.time() - t0 < d.get("timeout", 100.0):
                self.spin(0.5)
                self.narrate()
                if "reached" in self.status:
                    break
            took = time.time() - t0
            if "reached" in self.status:
                reached += 1
                extra = f", body {100 * (self.pos[2] - 0.19):+.0f} cm higher" if name == "stairs" else ""
                self.say(f"reached it in {took:.0f} s{extra}")
            else:
                self.say(f"didn't get there in {took:.0f} s")
        return reached, len(d["goals"])


def main():
    names = sys.argv[1:] or list(DEMOS)
    demo = Demo()
    results = {name: demo.run(name) for name in names}
    print("\n== summary: " + ", ".join(f"{k} {a}/{b}" for k, (a, b) in results.items()), flush=True)


if __name__ == "__main__":
    main()
