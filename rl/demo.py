"""Webots demos of what Bob can do. Run with the sim in planner mode:

    scripts/start_sim.sh --nav planner
    scripts/sim_ros.sh python3 /workspace/rl/demo.py            all of them, in order (~6 min)
    scripts/sim_ros.sh python3 /workspace/rl/demo.py tour       or one: tour, stairs, flip, walls, tall, ledge

  tour    walk to a run of targets round the arena, steering clear of the walls and boxes
  stairs  see the target on the landing of the east staircase, walk up it and reach it
  flip    turned upside down, right himself, then walk on to a target
  walls   a target hidden behind a wall: look, search, spot it and go round the wall to it
  tall    walked straight at the west staircase of three 10 cm steps (harder than he's mastered)
  ledge   walked straight at the 30 cm ledge in the south (above his hips: an experiment)
  chase   two Bobs. Bob2 collects green prizes that pop up at random open spots, dodging Bob on
          the way. Bob hunts Bob2 (he has to see him with his camera, and remembers where he
          last saw him). It ends when Bob gets within CATCH m, or after CHASE_TIME s. Needs
          chase mode: scripts/start_sim.sh --world chase (the world with Bob2, and his bridge).

Or from the Webots window (planner mode): click the 3D view and press 1 tour, 2 stairs, 3 flip,
4 walls, 5 tall, 6 ledge, 7 chase, 0 all, S stop; the bridge starts this script itself.
The navigator steers round anything over 8 cm, so tall and ledge drive him with /cmd_vel instead.

Each demo puts him in place first (/place), moves the orange target (/goal) and narrates what
the navigator says it's doing (/nav/status). The Webots camera follows him.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Float64, String

TITLES = {
    "tour": "walk to a run of targets round the arena, steering clear of the walls and boxes",
    "stairs": "see the target on the landing of the east staircase, walk up it and reach it",
    "flip": "turned upside down, right himself, then walk on to a target",
    "walls": "a target hidden behind a wall: look, search, spot it and go round the wall to it",
    "tall": "walked straight at three 10 cm steps",
    "ledge": "walked straight at a 30 cm ledge",
    "chase": "Bob2 collects prizes that pop up at random, dodging Bob; Bob hunts Bob2",
}
DEMOS = {
    "tour": dict(start=(0.0, 0.0, 0.0), goals=[(1.0, 0.5), (2.25, 1.8), (1.0, -0.6), (-0.5, -1.2), (-2.0, -1.2), (0.0, 0.0)]),
    "stairs": dict(start=(3.4, 0.55, -math.pi / 2), goals=[(3.35, -1.05)]),
    "flip": dict(start=(0.0, -0.5, 0.0), flip=180.0, goals=[(1.0, 0.5)]),
    "walls": dict(start=(0.0, 0.0, 0.0), goals=[(-1.6, -2.8)], timeout=150.0),
    # drive: (forward m/s, seconds, height (m) that counts as on top)
    "tall": dict(start=(-3.4, -1.3, math.pi / 2), drive=(0.2, 40.0, 0.27), goals=[]),
    "ledge": dict(start=(1.5, -2.5, -math.pi / 2), drive=(0.2, 40.0, 0.27), goals=[]),
    "chase": dict(start=(-1.5, -0.8, 0.0), runner=(0.8, 0.6, 0.0), goals=[]),
}
CHASE_TIME = 120.0   # s Bob2 has to survive
CATCH = 0.45         # m between their centres: caught (their bodies are ~0.3 m across)
RUNNER_PARK = (-3.4, 3.4, 0.0)   # where Bob2 waits between chases (cad/rocky_bob/build.py RUNNER_WBT)
FLEE_STEP = 1.0      # m: dodging, Bob2 picks his next spot this far away
SAFE = 1.5           # m: with Bob further than this (and not in the way), Bob2 goes straight for the prize
PRIZE_AWAY = 1.5     # m: a new prize appears at least this far from Bob2
COLLECT = 0.3        # m from a prize's post to his centre: collected
ARENA = 3.5          # m: spots stay this far inside the arena walls (it is 8 x 8 m)


def arena_boxes():
    """Footprints (x, y, half x, half y) of everything in the arena he can't run through or over,
    from cad/rocky_bob/build.py (stairs and steps count too: Bob2 shouldn't flee up them)."""
    import ast
    import pathlib
    src = pathlib.Path(__file__).resolve().parents[1] / "cad/rocky_bob/build.py"
    values = {n.targets[0].id: ast.literal_eval(n.value) for n in ast.parse(src.read_text()).body
              if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") in ("OBSTACLES", "STAIRS", "PLATFORMS")}
    out = []
    for name, x, y, sx, sy, h, yaw, _ in values["OBSTACLES"]:
        r = max(sx, sy) / 2 if abs(yaw) > 5 else None   # rotated: use a square around it
        out.append((x, y, r or sx / 2, r or sy / 2))
    for name, x, y, axis, n, rise, tread, landing, width in values["STAIRS"]:
        length = landing + 2 * n * tread
        out.append((x, y, length / 2, width / 2) if axis == "x" else (x, y, width / 2, length / 2))
    for name, x, y, sx, sy, h in values["PLATFORMS"]:
        out.append((x, y, sx / 2, sy / 2))
    return out


def clear(p, boxes, margin=0.35):
    return (abs(p[0]) < ARENA and abs(p[1]) < ARENA
            and all(abs(p[0] - x) > hx + margin or abs(p[1] - y) > hy + margin for x, y, hx, hy in boxes))


def segment_gap(a, b, p):
    """Distance from point p to the segment a-b."""
    ab = (b[0] - a[0], b[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / max(ab[0] ** 2 + ab[1] ** 2, 1e-9)))
    return math.dist(p, (a[0] + t * ab[0], a[1] + t * ab[1]))


def random_prize(rng, runner, hunter, boxes):
    """A random open spot for the next prize: at least PRIZE_AWAY from Bob2 and not right by Bob."""
    for _ in range(500):
        c = (rng.uniform(-ARENA, ARENA), rng.uniform(-ARENA, ARENA))
        if clear(c, boxes) and math.dist(c, runner) > PRIZE_AWAY and math.dist(c, hunter) > 1.0:
            return c
    return (0.0, 0.0)


def runner_waypoint(me, hunter, prize, boxes, current=None):
    """Where Bob2 heads next: straight for the prize while Bob is far and not in the way, else the open
    spot FLEE_STEP away that best trades getting closer to the prize against getting away from Bob
    (the closer Bob is, the more getting away counts). Keeps the current spot unless a new one is
    clearly better."""
    gap = math.dist(me, hunter)
    path_clear = all(clear((me[0] + f * (prize[0] - me[0]), me[1] + f * (prize[1] - me[1])), boxes, 0.25)
                     for f in (0.25, 0.5, 0.75))
    if gap > SAFE and segment_gap(me, prize, hunter) > 0.9 and path_clear:
        return prize
    danger = 2.5 if gap < 1.0 else 1.2

    def value(c):
        away = math.dist(c, hunter)
        past = gap - away   # > 0: running towards him
        room = sum(clear((c[0] + 0.8 * math.cos(k * math.pi / 4), c[1] + 0.8 * math.sin(k * math.pi / 4)), boxes)
                   for k in range(8))   # open ground around it, so he doesn't run into a corner
        return -math.dist(c, prize) + danger * away - 2.0 * max(past, 0.0) + 0.15 * room

    best, best_v = None, -1e9
    for k in range(24):
        a = 2 * math.pi * k / 24
        c = (me[0] + FLEE_STEP * math.cos(a), me[1] + FLEE_STEP * math.sin(a))
        if all(clear((me[0] + f * (c[0] - me[0]), me[1] + f * (c[1] - me[1])), boxes) for f in (0.25, 0.5, 0.75, 1.0)):
            if value(c) > best_v:
                best, best_v = c, value(c)
    if current is not None and current != prize and math.dist(current, me) > 0.25 and clear(current, boxes) \
            and (best is None or value(current) > best_v - 0.3):
        return current
    return best or me


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
        self.cmd_pub = self.n.create_publisher(Twist, "/cmd_vel", 5)
        self.track_pub = self.n.create_publisher(PointStamped, "/track", 5)
        self.prize_pub = self.n.create_publisher(PointStamped, "/prize", 5)
        self.runner_place = self.n.create_publisher(PointStamped, "/bob2/place", 5)
        self.runner_track = self.n.create_publisher(PointStamped, "/bob2/track", 5)
        self.runner_pos = None
        self.n.create_subscription(PointStamped, "/bob2/sim/position",
                                   lambda m: setattr(self, "runner_pos", (m.point.x, m.point.y, m.point.z)), 5)
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
        if "runner" in d:
            return self.chase(d["runner"])
        if "drive" in d:
            return self.drive(*d["drive"])
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


    def chase(self, runner_start):
        import random
        t0 = time.time()
        while self.runner_pos is None and time.time() - t0 < 5.0:   # his bridge may still be starting
            self.spin(0.2)
        if self.runner_pos is None:
            self.say("no Bob2 here: the chase needs chase mode (scripts/start_sim.sh --world chase)")
            return 0, 1
        rng, boxes = random.Random(), arena_boxes()
        for _ in range(2):
            self.runner_place.publish(self.point(*runner_start))
            self.runner_track.publish(self.point(*runner_start[:2]))
            self.spin(1.0)
        self.spin(1.0)
        prize = random_prize(rng, self.runner_pos[:2], self.pos[:2], boxes)
        self.prize_pub.publish(self.point(*prize))
        self.say(f"Bob2 goes for the prizes, Bob hunts him. Go! (first prize at ({prize[0]:+.1f}, {prize[1]:+.1f}))")
        self.goal_pub.publish(self.point(*self.runner_pos[:2]))   # a fresh target for Bob's planner
        spot, t0, next_spot, caught, prizes = None, time.time(), 0.0, False, 0
        closest = math.dist(self.pos[:2], self.runner_pos[:2])
        while time.time() - t0 < CHASE_TIME:
            self.spin(0.2)
            me, runner = self.pos[:2], self.runner_pos[:2]
            gap = math.dist(me, runner)
            self.track_pub.publish(self.point(*runner))     # Bob's orange target rides on Bob2
            if math.dist(runner, prize) < COLLECT:
                prizes += 1
                prize = random_prize(rng, runner, me, boxes)
                self.prize_pub.publish(self.point(*prize))
                self.say(f"Bob2 got prize {prizes}! next one at ({prize[0]:+.1f}, {prize[1]:+.1f})")
                next_spot, spot = 0.0, None
            if time.time() >= next_spot:
                next_spot = time.time() + 0.5
                new = runner_waypoint(runner, me, prize, boxes, spot)
                if new != spot:
                    spot = new
                    self.runner_track.publish(self.point(*spot))
            if gap < closest - 0.25:
                closest = gap
                self.say(f"... {gap:.1f} m apart")
            self.narrate()
            if gap < CATCH:
                caught = True
                break
        took = time.time() - t0
        self.say(f"caught! Bob got Bob2 after {took:.0f} s; Bob2 collected {prizes} prize(s)" if caught
                 else f"time's up: Bob2 survived {CHASE_TIME:.0f} s and collected {prizes} prize(s) (closest {closest:.1f} m)")
        self.goal_pub.publish(self.point(*self.pos[:2]))   # Bob stops where he is
        for _ in range(2):                                 # Bob2 and the prize back to his corner
            self.runner_place.publish(self.point(*RUNNER_PARK))
            self.runner_track.publish(self.point(*RUNNER_PARK[:2]))
            self.prize_pub.publish(self.point(RUNNER_PARK[0], RUNNER_PARK[1] + 0.5))
            self.spin(0.5)
        return prizes, int(not caught)

    def drive(self, speed, seconds, top):
        """Walk him straight ahead until his body is `top` higher than at the start, or time runs out."""
        z0, best = self.pos[2], 0.0
        self.say(f"walking him straight ahead at {speed} m/s")
        cmd = Twist()
        cmd.linear.x = speed
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.cmd_pub.publish(cmd)
            self.spin(0.1)
            rise = self.pos[2] - z0
            if rise > best + 0.02:
                best = rise
                self.say(f"... body {100 * best:.0f} cm higher")
            if rise >= top:
                break
        self.cmd_pub.publish(Twist())
        took = time.time() - t0
        ok = best >= top
        self.say(f"made it up in {took:.0f} s" if ok else f"didn't get up in {took:.0f} s (best {100 * best:.0f} cm higher)")
        return int(ok), 1


def main():
    names = sys.argv[1:] or list(DEMOS)
    demo = Demo()
    results = {name: demo.run(name) for name in names}
    print("\n== summary: " + ", ".join(f"{k} {a}/{b}" for k, (a, b) in results.items()), flush=True)


if __name__ == "__main__":
    main()
