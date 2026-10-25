"""Random rough terrain for training: bumps, low blocks, stairs and step-up platforms, by difficulty level.

The terrain is a MuJoCo height field (the `terrain` hfield in rocky_bob.xml). Level 0 is flat;
level 1 has bumps up to BUMP_HEIGHT, blocks up to BLOCK_HEIGHT, wide flat platforms up to
PLATFORM_HEIGHT, and stairs of five kinds with risers up to STAIR_RISE and treads of 24-45 cm:
pyramids (up and down on every side), straight flights (up, a landing, sometimes down, 0.8-2.5 m
wide), kerbs (long single steps), sunken stairs (down into a pit and up again), and in half of
them uneven steps, each its own height. A flat pad around the spawn point gives each episode a clean start.
The walker is blind: it learns to climb by feel, like the real robot.
"""

import numpy as np

BUMP_HEIGHT = 0.04       # m, peak-to-trough of the smooth bumps at level 1
BLOCK_HEIGHT = 0.07      # m, tallest obstacle block at level 1 (the 25 kg servos step ~4.5 cm)
BLOCKS = 400             # blocks scattered over the 12 x 12 m field at level 1 (~3 per m²)
SPAWN_PAD = (0.35, 0.7)  # m: flat inside the first radius, full roughness past the second
STAIRS = 6               # pyramid staircases on the field at level 1
STAIR_RISE = 0.05        # m, tallest riser at level 1 (risers are 40-100 % of this). 0.08 from scratch was too big a jump, and 0.06 (tried after he climbed 3-4 cm) stalled progress for 5 rounds
STAIR_TREAD = (0.20, 0.50)   # m: from tight (shorter than his stance) to deep
STAIR_STEPS = (1, 5)     # steps up each side (then a landing and the same down the far side)
PLATFORMS = 15           # wide flat step-ups at level 1
FLIGHTS = 8              # straight flights (up, landing, maybe down) at level 1
CURBS = 12               # long single steps (kerbs) at level 1
PITS = 3                 # sunken stairs (down into a pit and back up) at level 1
UNEVEN = 0.5             # share of staircases whose steps each get their own height (+-35 %)
PLATFORM_HEIGHT = 0.10
# Stair practice (env.STAIR_PRACTICE) puts one of these straight ahead of him, as (share, kind):
#   stairs: a flight like the field's, risers up to STAIR_RISE
#   tall:   1-3 steps of TALL_RISE, as high as his belly can clear with his legs straightened
#   ledge:  one LEDGE_RISE block to climb onto, up to 30 cm (above his hips: belly contact allowed)
# (30 % tall and 15 % ledges, all at once, cost him his ordinary stairs for seven rounds: so fewer,
# and the tall steps start at 6 cm and grow towards 10 cm as he gets over them, env.tall_level)
PRACTICE_KINDS = ((0.80, "stairs"), (0.15, "tall"), (0.05, "ledge"))
TALL_RISE = (0.06, 0.10)   # m: the tallest riser at tall_level 0 and 1
LEDGE_RISE = (0.12, 0.30)


def _rises(rng, level, n):
    """Riser heights for n steps: one height, or (UNEVEN of the time) each step its own."""
    rise = rng.uniform(0.4, 1.0) * STAIR_RISE * level
    if rng.random() < UNEVEN:
        return rise * rng.uniform(0.65, 1.35, n)
    return np.full(n, rise)


def _steps(rng, level):
    n = int(rng.integers(STAIR_STEPS[0], STAIR_STEPS[1] + 1))
    rises = _rises(rng, level, n)
    while rises.sum() > 0.3 and len(rises) > 1:   # stay inside the height field's range
        rises = rises[:-1]
    return rises, rng.uniform(*STAIR_TREAD)


def _local(X, Y, c, yaw):
    co, si = np.cos(yaw), np.sin(yaw)
    return co * (X - c[0]) + si * (Y - c[1]), -si * (X - c[0]) + co * (Y - c[1])


def _pyramid(rng, level, X, Y, half_x, half_y):
    """Square pyramid: steps up on every side to a landing."""
    c = rng.uniform(-half_x + 1, half_x - 1), rng.uniform(-half_y + 1, half_y - 1)
    u, v = _local(X, Y, c, rng.uniform(0, np.pi / 2))
    rises, tread = _steps(rng, level)
    landing = rng.uniform(0.3, 0.6)
    k = np.clip(np.floor((landing + len(rises) * tread - np.maximum(np.abs(u), np.abs(v))) / tread), -1, len(rises) - 1)
    tops = np.r_[0.0, np.cumsum(rises)]
    return tops[(k + 1).astype(int)]


def _flight(rng, level, X, Y, c, yaw, start_at_origin=False, steps=None):
    """Straight flight along yaw from c: up, a landing, and (half the time) down again, of random width.
    steps: (rises, tread) to use instead of random ones."""
    u, v = _local(X, Y, c, yaw)
    rises, tread = steps if steps is not None else _steps(rng, level)
    n, width, landing = len(rises), rng.uniform(0.6, 3.0), rng.uniform(0.3, 1.2)
    tops = np.r_[0.0, np.cumsum(rises)]
    up = np.clip(np.floor(u / tread) + 1, 0, n).astype(int)                    # steps climbed by u
    h = tops[up]
    if rng.random() < 0.5 or start_at_origin:                                   # and down the far side
        far = n * tread + landing
        down = np.clip(np.floor((u - far) / tread) + 1, 0, n).astype(int)
        h = np.where(u > far, tops[n - down], h)
        end = far + n * tread
    else:
        end = n * tread + landing
    return np.where((u >= 0) & (u < end) & (np.abs(v) < width / 2), h, 0.0)


def _curb(rng, level, X, Y, half_x, half_y):
    """A long single step: a raised strip to step up onto and off again."""
    c = rng.uniform(-half_x, half_x), rng.uniform(-half_y, half_y)
    u, v = _local(X, Y, c, rng.uniform(0, np.pi))
    length, depth = rng.uniform(1.0, 3.0), rng.uniform(0.3, 1.0)
    return np.where((np.abs(u) < length / 2) & (np.abs(v) < depth / 2), _rises(rng, level, 1)[0], 0.0)


def _pit(rng, level, X, Y, half_x, half_y, height):
    """Sunken stairs: steps down into a pit and back up the other side (cut into what's there)."""
    c = rng.uniform(-half_x + 1, half_x - 1), rng.uniform(-half_y + 1, half_y - 1)
    u, v = _local(X, Y, c, rng.uniform(0, np.pi / 2))
    rises, tread = _steps(rng, level)
    floor = rng.uniform(0.3, 0.6)
    k = np.clip(np.floor((floor + len(rises) * tread - np.maximum(np.abs(u), np.abs(v))) / tread), -1, len(rises) - 1)
    depth = np.r_[0.0, np.cumsum(rises)][(k + 1).astype(int)]
    # the height field can't go below 0, so raise the surroundings instead: a plateau with a pit in it
    inside = np.maximum(np.abs(u), np.abs(v)) < floor + len(rises) * tread + 0.6
    top = rises.sum()
    return np.where(inside, np.maximum(height, top - depth), height)


def practice_obstacle(rng, level, tall_level=1.0):
    """What stair practice puts ahead of him: (kind, distance to its first riser (m), steps).
    steps is None for an ordinary random flight, else (rises, tread). tall_level (0..1) sets how
    tall the tall steps are: risers up to TALL_RISE[0] at 0, up to TALL_RISE[1] at 1, mostly near the top."""
    shares, kinds = zip(*PRACTICE_KINDS)
    kind = kinds[int(rng.choice(len(kinds), p=np.array(shares) / sum(shares)))]
    d = rng.uniform(0.8, 1.2)
    if kind == "tall":
        n = int(rng.integers(1, 4))
        cap = TALL_RISE[0] + (TALL_RISE[1] - TALL_RISE[0]) * float(np.clip(tall_level, 0, 1))
        rises = rng.uniform(cap - 0.01, cap) * (rng.uniform(0.85, 1.0, n) if rng.random() < UNEVEN else np.ones(n))
        return kind, d, (rises, rng.uniform(0.30, 0.50))
    if kind == "ledge":
        return kind, d, (np.array([rng.uniform(*LEDGE_RISE)]), rng.uniform(0.5, 1.0))
    return kind, d, None


def generate(rng, level, nrow, ncol, half_x, half_y, max_height, ahead_yaw=None, ahead=None):
    """Height field values in [0, 1] (MuJoCo scales them by max_height).
    ahead_yaw: also put a practice obstacle just ahead of the spawn point, facing that way:
    ahead = practice_obstacle(...), or an ordinary flight of stairs if None."""
    if level <= 0 and ahead_yaw is None:
        return np.zeros((nrow, ncol))
    xs = np.linspace(-half_x, half_x, ncol)
    ys = np.linspace(-half_y, half_y, nrow)
    X, Y = np.meshgrid(xs, ys)

    # smooth bumps: a coarse random grid, bilinearly upsampled (~0.4 m features)
    coarse = rng.uniform(0, 1, (int(2 * half_y / 0.4) + 2, int(2 * half_x / 0.4) + 2))
    cy = (Y + half_y) / 0.4
    cx = (X + half_x) / 0.4
    y0, x0 = np.floor(cy).astype(int), np.floor(cx).astype(int)
    fy, fx = cy - y0, cx - x0
    bumps = (coarse[y0, x0] * (1 - fx) * (1 - fy) + coarse[y0, x0 + 1] * fx * (1 - fy)
             + coarse[y0 + 1, x0] * (1 - fx) * fy + coarse[y0 + 1, x0 + 1] * fx * fy)
    height = bumps * BUMP_HEIGHT * level

    # obstacles: low rectangular blocks
    for _ in range(int(BLOCKS * level)):
        bx, by = rng.uniform(-half_x, half_x), rng.uniform(-half_y, half_y)
        w, d = rng.uniform(0.05, 0.3, 2)
        h = rng.uniform(0.3, 1.0) * BLOCK_HEIGHT * level
        mask = (np.abs(X - bx) < w / 2) & (np.abs(Y - by) < d / 2)
        height[mask] = np.maximum(height[mask], h + bumps[mask] * BUMP_HEIGHT * level)

    # stairs, in several kinds (see the helpers below)
    for _ in range(int(round(STAIRS * level))):
        height = np.maximum(height, _pyramid(rng, level, X, Y, half_x, half_y))
    for _ in range(int(round(FLIGHTS * level))):
        c = rng.uniform(-half_x + 1.5, half_x - 1.5), rng.uniform(-half_y + 1.5, half_y - 1.5)
        height = np.maximum(height, _flight(rng, level, X, Y, c, rng.uniform(-np.pi, np.pi)))
    for _ in range(int(round(CURBS * level))):
        height = np.maximum(height, _curb(rng, level, X, Y, half_x, half_y))
    for _ in range(int(round(PITS * level))):
        height = _pit(rng, level, X, Y, half_x, half_y, height)
    if ahead_yaw is not None:   # a staircase right in his path (stair practice episodes)
        _, d, steps = ahead if ahead is not None else ("stairs", rng.uniform(0.8, 1.2), None)
        height = np.maximum(height, _flight(rng, max(level, 0.5), X, Y, (d * np.cos(ahead_yaw), d * np.sin(ahead_yaw)),
                                            ahead_yaw, start_at_origin=True, steps=steps))

    # platforms: wide flat boxes to step up onto
    for _ in range(int(round(PLATFORMS * level))):
        bx, by = rng.uniform(-half_x, half_x), rng.uniform(-half_y, half_y)
        w, d = rng.uniform(0.5, 1.4, 2)
        h = rng.uniform(0.3, 1.0) * PLATFORM_HEIGHT * level
        mask = (np.abs(X - bx) < w / 2) & (np.abs(Y - by) < d / 2)
        height[mask] = np.maximum(height[mask], h)

    r = np.hypot(X, Y)
    fade = np.clip((r - SPAWN_PAD[0]) / (SPAWN_PAD[1] - SPAWN_PAD[0]), 0, 1)
    height *= fade * fade * (3 - 2 * fade)
    return np.clip(height / max_height, 0, 1)


def heights_at(field, xs, ys, nrow, ncol, half_x, half_y, max_height):
    """Terrain heights (m) under arrays of points, bilinear."""
    cx = np.clip((np.asarray(xs) + half_x) / (2 * half_x) * (ncol - 1), 0, ncol - 1.001)
    cy = np.clip((np.asarray(ys) + half_y) / (2 * half_y) * (nrow - 1), 0, nrow - 1.001)
    c0, r0 = cx.astype(int), cy.astype(int)
    fx, fy = cx - c0, cy - r0
    z = (field[r0, c0] * (1 - fx) * (1 - fy) + field[r0, c0 + 1] * fx * (1 - fy)
         + field[r0 + 1, c0] * (1 - fx) * fy + field[r0 + 1, c0 + 1] * fx * fy)
    return z * max_height


def height_at(field, x, y, nrow, ncol, half_x, half_y, max_height):
    """Terrain height (m) under (x, y), bilinear."""
    cx = np.clip((x + half_x) / (2 * half_x) * (ncol - 1), 0, ncol - 1.001)
    cy = np.clip((y + half_y) / (2 * half_y) * (nrow - 1), 0, nrow - 1.001)
    c0, r0 = int(cx), int(cy)
    fx, fy = cx - c0, cy - r0
    z = (field[r0, c0] * (1 - fx) * (1 - fy) + field[r0, c0 + 1] * fx * (1 - fy)
         + field[r0 + 1, c0] * (1 - fx) * fy + field[r0 + 1, c0 + 1] * fx * fy)
    return z * max_height
