"""Random rough terrain for training: smooth bumps plus low blocks, scaled by a difficulty level.

The terrain is a MuJoCo height field (the `terrain` hfield in rocky_bob.xml). Level 0 is flat;
level 1 has bumps up to BUMP_HEIGHT and blocks up to BLOCK_HEIGHT. A flat pad around the spawn
point gives each episode a clean start.
"""

import numpy as np

BUMP_HEIGHT = 0.04       # m, peak-to-trough of the smooth bumps at level 1
BLOCK_HEIGHT = 0.07      # m, tallest obstacle block at level 1 (the 25 kg servos step ~4.5 cm)
BLOCKS = 400             # blocks scattered over the 12 x 12 m field at level 1 (~3 per m²)
SPAWN_PAD = (0.35, 0.7)  # m: flat inside the first radius, full roughness past the second


def generate(rng, level, nrow, ncol, half_x, half_y, max_height):
    """Height field values in [0, 1] (MuJoCo scales them by max_height)."""
    if level <= 0:
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
