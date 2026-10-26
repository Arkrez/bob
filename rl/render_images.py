"""Pictures of Bob for the README (docs/images/): the CAD parts, and MuJoCo renders of him standing,
and on the training terrain (Webots screenshots: the bridge's /screenshot topic).

    .venv/bin/python render_images.py            writes docs/images/*.png
"""

import pathlib
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from env import RockyBobWalk

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE.parent / "docs" / "images"
STL = HERE.parent / "cad" / "rocky_bob" / "stl"
PARTS = [("rock_base", "rock base"), ("rock_lid", "rock lid (lifts off)"), ("mustache", "mustache"),
         ("hip_pod", "hip pod"), ("coxa", "coxa"), ("femur", "femur"), ("tibia", "tibia"), ("foot", "foot")]
STONE = (0.62, 0.57, 0.51)


def read_stl(path):
    """Triangles (n, 3, 3) from a binary or ASCII STL."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = [list(map(float, line.split()[1:])) for line in data.decode().splitlines() if line.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(float)


def shade(tris, light=(0.4, -0.5, 0.75)):
    """Flat Lambert shading of each triangle in STONE."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    k = 0.35 + 0.65 * np.clip(n @ (np.array(light) / np.linalg.norm(light)), 0, 1)
    return np.clip(np.outer(k, STONE), 0, 1)


def parts_sheet():
    fig = plt.figure(figsize=(16, 8), facecolor="white")
    for i, (name, title) in enumerate(PARTS):
        tris = read_stl(STL / f"{name}.stl")
        ax = fig.add_subplot(2, 4, i + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors=shade(tris), edgecolors="none"))
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=24, azim=-55)
        ax.set_axis_off()
        size = hi - lo
        ax.set_title(f"{title}\n{size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm", fontsize=12)
    fig.suptitle("Rocky-Bob printed parts (cad/rocky_bob/stl)", fontsize=15)
    fig.tight_layout()
    fig.savefig(OUT / "cad_parts.png", dpi=110)
    plt.close(fig)


def render(model, data, renderer, lookat, distance, azimuth, elevation):
    model.vis.map.znear = 0.0005   # relative to the 12 m terrain: the default clips a close-up of him
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, distance, azimuth, elevation
    renderer.update_scene(data, camera=cam)
    return renderer.render()


def save(img, name):
    plt.imsave(OUT / name, img)
    print(f"wrote docs/images/{name}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    parts_sheet()
    print("wrote docs/images/cad_parts.png")

    # standing on flat ground
    e = RockyBobWalk(randomize=False, terrain_level=0.0, command=(0, 0, 0), pushes=False)
    e.reset(seed=0)
    r = mujoco.Renderer(e.model, 720, 1280)
    for _ in range(200):
        e.step(np.zeros(12))
    p = e.data.qpos[:3].copy()
    save(render(e.model, e.data, r, p + [0, 0, 0.0], 0.9, 145, -20), "mujoco_standing.png")

    # the training terrain at full difficulty
    e = RockyBobWalk(randomize=False, terrain_level=1.0, command=(0, 0, 0), pushes=False)
    e.reset(seed=3)
    r = mujoco.Renderer(e.model, 720, 1280)
    mujoco.mjr_uploadHField(e.model, r._mjr_context, 0)
    for _ in range(100):
        e.step(np.zeros(12))
    save(render(e.model, e.data, r, e.data.qpos[:3] + [0.6, 0, 0], 4.0, 120, -30), "mujoco_terrain.png")


if __name__ == "__main__":
    main()
