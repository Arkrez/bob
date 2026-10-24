"""Draw a sim_watch.py recording: the arena seen from above (walls and tall boxes dark, low steps,
stairs and platforms the lidar can't see in orange), Bob's path coloured by time, where he was
when the status changed, the target, and his camera's view.

    rl/.venv/bin/python rl/plot_watch.py      -> rl/runs/sim_watch.png
"""

import ast
import json
import math
import pathlib

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
BUILD = HERE.parent / "cad/rocky_bob/build.py"
LIDAR_PLANE = 0.36   # m above the floor


def world():
    """OBSTACLES, STAIRS and PLATFORMS from cad/rocky_bob/build.py, without importing CadQuery."""
    tree = ast.parse(BUILD.read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ("OBSTACLES", "STAIRS", "PLATFORMS"):
            out[node.targets[0].id] = ast.literal_eval(node.value)
    return out


def box(ax, x, y, sx, sy, yaw_deg, color, alpha, label=None):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    pts = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
    ax.add_patch(Polygon(pts, closed=True, color=color, alpha=alpha))
    if label:
        ax.text(x, y, label, ha="center", va="center", fontsize=7)


def main():
    track = json.loads((HERE / "runs/sim_watch.json").read_text())
    w = world()
    fig = plt.figure(figsize=(14, 7.5))
    ax = fig.add_axes([0.03, 0.05, 0.6, 0.9])
    ax.set_xlim(-4, 4); ax.set_ylim(-4, 4); ax.set_aspect("equal"); ax.grid(alpha=0.2)
    for name, x, y, sx, sy, h, yaw, _ in w["OBSTACLES"]:
        tall = h > LIDAR_PLANE
        box(ax, x, y, sx, sy, yaw, "#333" if tall else "#e8832a", 0.85 if tall else 0.5, f"{name}\n{h*100:.0f} cm")
    for name, x, y, axis, n, rise, tread, landing, width in w["STAIRS"]:
        for i in range(1, n + 1):
            length = landing + 2 * (n - i + 1) * tread
            sx, sy = (length, width) if axis == "x" else (width, length)
            box(ax, x, y, sx, sy, 0, "#e8832a", 0.18)
        ax.text(x, y, f"{name}\n{n} x {rise*100:.1f} cm", ha="center", va="center", fontsize=7)
    for name, x, y, sx, sy, h in w["PLATFORMS"]:
        box(ax, x, y, sx, sy, 0, "#e8832a", 0.5, f"{name}\n{h*100:.0f} cm")
    xs, ys = np.array([p["x"] for p in track]), np.array([p["y"] for p in track])
    sc = ax.scatter(xs, ys, c=[p["t"] for p in track], cmap="viridis", s=8, zorder=3)
    plt.colorbar(sc, ax=ax, label="s", fraction=0.03)
    last = None
    for p in track:
        if p["status"] != last:
            ax.annotate(p["status"][:28], (p["x"], p["y"]), fontsize=6, alpha=0.8, xytext=(4, 4), textcoords="offset points")
            last = p["status"]
        if p["target"]:
            ax.plot(*p["target"], "r*", ms=12, zorder=4)
    p = track[-1]
    ax.arrow(p["x"], p["y"], 0.3 * math.cos(p["yaw"]), 0.3 * math.sin(p["yaw"]), width=0.04, color="k", zorder=5)
    ax.set_title(f"{track[0]['t']:.0f}-{p['t']:.0f} s | dark: walls the lidar sees, orange: low things it can't | now: {p['status'][:40]}", fontsize=9)
    cam = HERE / "runs/sim_watch_camera.npy"
    if cam.exists():
        a2 = fig.add_axes([0.66, 0.35, 0.32, 0.3])
        a2.imshow(np.load(cam)); a2.set_axis_off(); a2.set_title("his camera, at the end", fontsize=9)
    fig.savefig(HERE / "runs/sim_watch.png", dpi=110)
    print("wrote rl/runs/sim_watch.png")


if __name__ == "__main__":
    main()
