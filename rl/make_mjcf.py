"""Generate rl/rocky_bob.xml (MuJoCo) from cad/rocky_bob/sim/rocky_bob.urdf.

Re-run after changing the CAD (cad/rocky_bob/build.py) so training matches the robot:
    .venv/bin/python make_mjcf.py

Servo model (spec.py): a MuJoCo position actuator, force clamped to the servo's stall torque.
Collision: the URDF's primitive shapes. The legs collide with the floor only, not with each
other or the body (the CAD check shows the legs can't reach the body anyway).
"""

import math
import pathlib
import xml.etree.ElementTree as ET

from spec import ARMATURE, FOOT_FRICTION, JOINT_DAMPING, JOINT_FRICTION as FRICTIONLOSS, STALL_TORQUE
from spec import PHYSICS_DT, SERVO_KP as KP, SERVO_KV as KV
from lidar import LIDAR_Z
from tof import POS as TOF_POS
from nav_spec import OBSTACLES

HERE = pathlib.Path(__file__).resolve().parent
URDF = HERE.parent / "cad/rocky_bob/sim/rocky_bob.urdf"
OUT = HERE / "rocky_bob.xml"

# servo model
# servo model constants live in spec.py so the Webots bridge can reproduce them

# terrain height field (see terrain.py); flat until env.py fills it in
TERRAIN = dict(nrow=480, ncol=480, half=6.0, max_height=0.35, base=0.05)   # 2.5 cm cells: steep risers; 0.35: room for stairs

# collision bits: everything hits the floor, and the legs hit the body and each other, so the only
# limits on a leg are its servos and the robot's own shape (MuJoCo skips each link and its parent)
FLOOR = 'contype="1" conaffinity="3"'
BODY = 'contype="2" conaffinity="5"'
LEG = 'contype="4" conaffinity="7"'


def floats(s):
    return [float(v) for v in s.split()]


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def load():
    root = ET.parse(URDF).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    children = {}
    for j in root.findall("joint"):
        children.setdefault(j.find("parent").get("link"), []).append(j)
    return links, children


def geoms(link, is_body, meshes, materials):
    out = []
    for v in link.findall("visual"):
        mesh = pathlib.Path(v.find("geometry/mesh").get("filename")).stem
        meshes.add(mesh)
        rgba = v.find("material/color").get("rgba")
        materials[mesh] = rgba
        out.append(f'<geom class="visual" type="mesh" mesh="{mesh}" material="{mesh}"/>')
    bits = BODY if is_body else LEG
    for c in link.findall("collision"):
        o = c.find("origin")
        pos = fmt(floats(o.get("xyz")))
        rpy = floats(o.get("rpy"))
        g = c.find("geometry")[0]
        if g.tag == "mesh":   # convex: MuJoCo collides with its convex hull
            mesh = pathlib.Path(g.get("filename")).stem
            meshes.add(mesh)
            shape = f'type="mesh" mesh="{mesh}"'
        elif g.tag == "box":
            size = fmt([s / 2 for s in floats(g.get("size"))])
            shape = f'type="box" size="{size}"'
        elif g.tag == "cylinder":
            shape = f'type="cylinder" size="{float(g.get("radius")):.6g} {float(g.get("length")) / 2:.6g}"'
        else:
            shape = f'type="sphere" size="{float(g.get("radius")):.6g}"'
        name = f'name="{c.get("name")}" ' if c.get("name") else ""
        if g.tag == "sphere":
            name = f'name="{link.get("name").split("_")[0]}_foot" '
        out.append(f'<geom {name}class="collision" {shape} pos="{pos}" euler="{fmt(rpy)}" {bits}/>')
    i = link.find("inertial")
    if i is not None:
        com = fmt(floats(i.find("origin").get("xyz")))
        m = float(i.find("mass").get("value"))
        I = i.find("inertia")
        full = fmt([float(I.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")])
        out.insert(0, f'<inertial pos="{com}" mass="{m:.6g}" fullinertia="{full}"/>')
    return out


def body_xml(name, links, children, meshes, materials, depth):
    pad = "  " * depth
    lines = []
    for g in geoms(links[name], name == "base_link", meshes, materials):
        lines.append(pad + g)
    for j in children.get(name, []):
        child = j.find("child").get("link")
        o = j.find("origin")
        pos = fmt(floats(o.get("xyz")))
        rpy = floats(o.get("rpy")) if o.get("rpy") else [0, 0, 0]
        inner = []
        if j.get("type") == "revolute":
            inner.append(f'<joint name="{j.get("name")}" class="servo" axis="{j.find("axis").get("xyz")}"/>')
        if not links[child].findall("visual") and not links[child].findall("collision"):
            # empty URDF link (the foot frame): keep it as a site
            lines.append(pad + f'<site name="{child}" pos="{pos}" size="0.005"/>')
            continue
        lines.append(pad + f'<body name="{child}" pos="{pos}" euler="{fmt(rpy)}">')
        lines += [pad + "  " + s for s in inner]
        lines += body_xml(child, links, children, meshes, materials, depth + 1)
        lines.append(pad + "</body>")
    return lines


def main():
    links, children = load()
    meshes, materials = set(), {}
    tree = body_xml("base_link", links, children, meshes, materials, 3)
    joint_names = [f"joint{j}_leg{l}" for l in range(4) for j in (1, 3, 4)]
    meshdir = pathlib.Path("..") / "cad/rocky_bob/sim/meshes"

    xml = f"""<!-- Generated by make_mjcf.py from cad/rocky_bob/sim/rocky_bob.urdf. Do not edit by hand. -->
<mujoco model="rocky_bob">
  <compiler angle="radian" meshdir="{meshdir}" autolimits="true"/>
  <option timestep="{PHYSICS_DT}" integrator="implicitfast"/>

  <default>
    <default class="visual"><geom contype="0" conaffinity="0" group="2"/></default>
    <default class="collision"><geom group="3" friction="{FOOT_FRICTION} 0.02 0.002" condim="3" rgba="0.8 0.3 0.3 0.3"/></default>
    <default class="servo">
      <joint range="-1.5708 1.5708" damping="{JOINT_DAMPING}" armature="{ARMATURE}" frictionloss="{FRICTIONLOSS}"/>
      <position kp="{KP}" kv="{KV}" forcerange="-{STALL_TORQUE} {STALL_TORQUE}" ctrlrange="-1.5708 1.5708"/>
    </default>
  </default>

  <asset>
{chr(10).join(f'    <mesh name="{m}" file="{m}.stl"/>' for m in sorted(meshes))}
{chr(10).join(f'    <material name="{m}" rgba="{rgba}"/>' for m, rgba in sorted(materials.items()))}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.35 0.37 0.40" rgb2="0.28 0.30 0.33" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="48 48" texuniform="false" reflectance="0.05"/>
    <hfield name="terrain" nrow="{TERRAIN['nrow']}" ncol="{TERRAIN['ncol']}" size="{TERRAIN['half']} {TERRAIN['half']} {TERRAIN['max_height']} {TERRAIN['base']}"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.65 0.8" rgb2="0.1 0.1 0.15" width="256" height="256"/>
  </asset>

  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/></visual>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="hfield" hfield="terrain" material="grid" {FLOOR} friction="{FOOT_FRICTION} 0.02 0.002"/>
    <body name="base_link" pos="0 0 0.2">
      <freejoint name="root"/>
      <site name="imu" pos="0 0 0.05" size="0.01"/>
      <site name="lidar" pos="0 0 {LIDAR_Z}" size="0.01"/>
      <site name="tof" pos="{fmt(TOF_POS)}" size="0.005"/>
      <camera name="track" mode="trackcom" pos="0.6 -0.9 0.5" xyaxes="0.83 0.56 0 -0.2 0.3 0.93"/>
{chr(10).join(tree)}
    </body>
    <!-- Obstacles for navigation training: parked out of the way; nav_env.py moves them into place.
         They collide with the robot's body and legs (contype/conaffinity bits above). -->
{chr(10).join(f'    <body name="obstacle{i}" mocap="true" pos="{60 + 3 * i} 60 -5"><geom name="obstacle{i}" type="box" size="{fmt(h)}" contype="1" conaffinity="6" rgba="0.70 0.62 0.50 1"/></body>' for i, h in enumerate(OBSTACLES))}
    <!-- The target post: seen by the lidar (rays hit any geom) but nothing collides with it -->
    <body name="goal_post" mocap="true" pos="60 50 -5"><geom name="goal_post" type="cylinder" size="0.02 0.25" pos="0 0 0.25" contype="0" conaffinity="0" rgba="1 0.5 0.1 1"/></body>
  </worldbody>

  <actuator>
{chr(10).join(f'    <position name="{j}" joint="{j}" class="servo"/>' for j in joint_names)}
  </actuator>

  <sensor>
    <framequat name="orientation" objtype="site" objname="imu"/>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
    <velocimeter name="velocity" site="imu"/>
  </sensor>
</mujoco>
"""
    OUT.write_text(xml)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
