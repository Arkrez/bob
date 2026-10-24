"""Build Rocky-Bob: the rock body plus four rock legs, and export everything.

  step/   rocky_bob_assembly.step (zero pose), rocky_bob_standing.step, one STEP per printed part
  stl/    printable STLs in millimetres (print the leg parts x4)
  sim/    link-frame meshes in metres, rocky_bob.urdf, protos/RockyBob.proto, worlds/rocky_bob.wbt

Run:  python build.py      (needs `pip install cadquery`)
"""

import math
import pathlib
import sys

import cadquery as cq
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

import body
import leg

HERE = pathlib.Path(__file__).resolve().parent
# Servo, joint and contact constants shared with the RL training model (rl/spec.py), so
# Webots and MuJoCo simulate the same robot.
sys.path.insert(0, str(HERE.parents[1] / "rl"))
import lidar  # noqa: E402
import spec  # noqa: E402
COLORS = {**leg.COLORS, **body.COLORS}
STANDING = (0.0, 0.75, 1.05)   # joint1, joint3, joint4 (rad); matches rl/spec.py STAND
LEG_LINKS = ("mount", "coxa", "femur", "tibia")
URDF_LINK = {"mount": "mount", "coxa": "joint1_link", "femur": "joint2_link", "tibia": "joint3_link"}


def leg_link_locations(i, q=(0, 0, 0)):
    """Body-frame location of each link frame of leg i at joint angles q."""
    hip = body.hip_location(i)
    t = {"mount": hip, "coxa": hip * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), math.degrees(q[0]))}
    t["femur"] = t["coxa"] * cq.Location(cq.Vector(leg.L1, 0, 0)) * \
        cq.Location(cq.Vector(), cq.Vector(0, 1, 0), math.degrees(q[1]))
    t["tibia"] = t["femur"] * cq.Location(cq.Vector(leg.L2, 0, 0)) * \
        cq.Location(cq.Vector(), cq.Vector(0, 1, 0), math.degrees(q[2]))
    return t


def assembly(body_parts, leg_parts, q):
    asm = cq.Assembly(name="rocky_bob")
    for name, (shape, color, _) in body_parts.items():
        asm.add(shape, name=name, color=cq.Color(*COLORS[color]))
    for i in body.LEG_ANGLES:
        locs = leg_link_locations(i, q)
        for name, (lk, shape, color, _) in leg_parts.items():
            asm.add(shape, name=f"leg{i}_{name}", loc=locs[lk], color=cq.Color(*COLORS[color]))
    return asm


# ================================================================ mass properties
def props(items):
    """items: (shape, density kg/m^3 or None, mass kg or None). Returns mass, com (m), inertia (kg m^2)."""
    total = GProp_GProps()
    for shape, density, mass in items:
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape.wrapped, p)
        if density is None:
            density = mass / (p.Mass() * 1e-9)
        total.Add(p, density * 1e-9)
    c = total.CentreOfMass()
    I = total.MatrixOfInertia()
    return (total.Mass(), (c.X() / 1000, c.Y() / 1000, c.Z() / 1000),
            [[I.Value(r, k) * 1e-6 for k in (1, 2, 3)] for r in (1, 2, 3)])


def leg_link_props(leg_parts, lk):
    return props([(s, leg.PRINT_DENSITY if printed else None, None if printed else leg.SERVO_MASS)
                  for name, (l, s, _, printed) in leg_parts.items() if l == lk and not name.startswith("horn")])


def body_props(body_parts):
    return props([(s, body.PRINT_DENSITY if kind == "printed" else None,
                   None if kind == "printed" else body.ELECTRONICS_MASS[name])
                  for name, (s, _, kind) in body_parts.items()])


# ================================================================ collision primitives (m, link frame)
def leg_collisions(yt):
    m = 0.001
    yc = (-leg.G_IDLER + leg.TAB_TOP) / 2
    return {
        "mount": [("box", (0.065, 0.045, 0.056), (-0.020, 0, -0.010))],
        "coxa": [("box", (0.130, 0.055, 0.040), (0.065, -0.010, 0))],
        "femur": [("cyl", (leg.FEMUR_R * m, (leg.L2 + 32 - 30) * m), ((30 + leg.L2 + 32) / 2 * m, yc * m, 0))],
        "tibia": [("cyl", (leg.TIBIA_R * m, (leg.L3 - leg.FOOT_R - 40) * m), ((40 + leg.L3 - leg.FOOT_R) / 2 * m, yt * m, 0)),
                  ("sphere", (leg.FOOT_R * m,), (leg.L3 * m, yt * m, 0))],
    }


def body_collision(body_parts):
    """The rock's own convex shell (it rolls like the real one), plus the lidar plinth and the lidar,
    which is the highest point: upside down he lands on it, so it is its own named geom ("lidar")."""
    q = body.LIDAR_PLINTH / 2
    return [("mesh", "rock_collision", (0, 0, 0)),
            ("box", (2 * q / 1000, 2 * q / 1000, (body.LIDAR_TOP - 112) / 1000), (0, 0, (body.LIDAR_TOP + 112) / 2000)),
            ("box", (body.LIDAR_SIZE / 1000, body.LIDAR_SIZE / 1000, body.LIDAR_H / 1000),
             (0, 0, (body.LIDAR_TOP - body.LIDAR_POCKET_D + body.LIDAR_H / 2) / 1000), "lidar")]


# ================================================================ URDF
def f3(v):
    return " ".join(f"{x:.5f}" for x in v)


def urdf_geom(kind, dims):
    if kind == "mesh":
        return f'<mesh filename="meshes/{dims}.stl"/>'
    if kind == "box":
        return f'<box size="{f3(dims)}"/>'
    if kind == "cyl":
        return f'<cylinder radius="{dims[0]:.5f}" length="{dims[1]:.5f}"/>'
    return f'<sphere radius="{dims[0]:.5f}"/>'


def urdf_link(name, visuals, cols, inertial):
    m, c, I = inertial
    lines = [f'  <link name="{name}">']
    for mesh, rgb in visuals:
        lines.append(f'    <visual><geometry><mesh filename="meshes/{mesh}.stl"/></geometry>'
                     f'<material name="{mesh}"><color rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"/></material></visual>')
    for kind, dims, o, *name in cols:
        rpy = "0 1.5708 0" if kind == "cyl" else "0 0 0"
        named = f' name="{name[0]}"' if name else ""
        lines.append(f'    <collision{named}><origin xyz="{f3(o)}" rpy="{rpy}"/><geometry>{urdf_geom(kind, dims)}</geometry></collision>')
    lines.append(f'    <inertial><origin xyz="{f3(c)}"/><mass value="{m:.4f}"/>'
                 f'<inertia ixx="{I[0][0]:.3e}" iyy="{I[1][1]:.3e}" izz="{I[2][2]:.3e}" '
                 f'ixy="{I[0][1]:.3e}" ixz="{I[0][2]:.3e}" iyz="{I[1][2]:.3e}"/></inertial>')
    lines.append("  </link>")
    return "\n".join(lines)


def write_urdf(body_parts, leg_parts, yt):
    lines = ['<?xml version="1.0"?>',
             f'<!-- Generated by build.py: Rocky-Bob, rock body + 4 legs, {spec.SERVO_NAME} servos. -->',
             '<robot name="rocky_bob">',
             urdf_link("base_link", [(n, COLORS[c]) for n, (_, c, _) in body_parts.items()],
                       body_collision(body_parts), body_props(body_parts))]
    cols = leg_collisions(yt)
    for i, deg in body.LEG_ANGLES.items():
        hip = body.hip_location(i).toTuple()[0]
        for lk in LEG_LINKS:
            lines.append(urdf_link(f"leg{i}_{URDF_LINK[lk]}",
                                   [(n, COLORS[c]) for n, (l, _, c, _) in leg_parts.items() if l == lk],
                                   cols[lk], leg_link_props(leg_parts, lk)))
        lines.append(f'  <joint name="leg{i}_mount_fixed" type="fixed"><parent link="base_link"/><child link="leg{i}_mount"/>'
                     f'<origin xyz="{f3(v / 1000 for v in hip)}" rpy="0 0 {math.radians(deg):.6f}"/></joint>')
        for jn, parent, child, xyz, axis in (
                ("joint1", "mount", "coxa", (0, 0, 0), "0 0 1"),
                ("joint3", "coxa", "femur", (leg.L1, 0, 0), "0 1 0"),
                ("joint4", "femur", "tibia", (leg.L2, 0, 0), "0 1 0")):
            lines.append(f'  <joint name="{jn}_leg{i}" type="revolute"><parent link="leg{i}_{URDF_LINK[parent]}"/>'
                         f'<child link="leg{i}_{URDF_LINK[child]}"/><origin xyz="{f3(v / 1000 for v in xyz)}"/>'
                         f'<axis xyz="{axis}"/><limit lower="-1.5708" upper="1.5708" effort="{spec.STALL_TORQUE}" '
                         f'velocity="{spec.SERVO_FREE_SPEED}"/></joint>')
        lines.append(f'  <link name="leg{i}_foot"/>')
        lines.append(f'  <joint name="leg{i}_foot_fixed" type="fixed"><parent link="leg{i}_joint3_link"/>'
                     f'<child link="leg{i}_foot"/><origin xyz="{leg.L3 / 1000:.5f} {yt / 1000:.5f} 0"/></joint>')
    lines.append("</robot>")
    (HERE / "sim/rocky_bob.urdf").write_text("\n".join(lines) + "\n")


# ================================================================ Webots PROTO
# Webots arena furniture: (name, x, y, size x, size y, height, yaw deg, colour). Walls and two
# furniture-height boxes reach above the lidar's scan plane (~36 cm up when standing) so it sees
# them; the small box and rocks are below it, like the clutter a 2D lidar misses in a real room.
# Low steps he can climb (<= 6 cm). The spawn point and the starting target stay clear.
OBSTACLES = [
    ("wall_west", -1.25, 1.5, 2.5, 0.06, 0.60, 0, (0.75, 0.72, 0.68)),
    ("wall_east", 2.0, -1.25, 0.06, 2.5, 0.60, 0, (0.75, 0.72, 0.68)),
    ("wall_south", -1.6, -2.2, 1.6, 0.06, 0.60, 0, (0.75, 0.72, 0.68)),
    ("box_a", 1.8, 1.8, 0.30, 0.30, 0.45, 15, (0.55, 0.35, 0.20)),
    ("box_b", -2.4, -0.5, 0.25, 0.40, 0.50, -10, (0.55, 0.35, 0.20)),
    ("box_c", 0.6, -2.6, 0.20, 0.20, 0.15, 30, (0.55, 0.35, 0.20)),
    ("step_low", -1.2, 0.4, 0.60, 0.60, 0.04, 0, (0.45, 0.45, 0.50)),
    ("ledge", 2.9, 0.9, 0.50, 1.00, 0.06, 0, (0.45, 0.45, 0.50)),
    ("rock_a", 3.0, -2.6, 0.22, 0.18, 0.12, 35, (0.42, 0.40, 0.37)),
    ("rock_b", -3.0, 2.8, 0.28, 0.20, 0.16, -20, (0.42, 0.40, 0.37)),
    ("rock_c", 1.2, -1.6, 0.16, 0.14, 0.09, 60, (0.42, 0.40, 0.37)),
]


# Stairs to climb: (name, centre x, y, along "x" or "y", steps each way, riser m, tread m, landing m, width m).
# Up one side, a landing, down the other, like the pyramid stairs he trains on (rl/terrain.py).
# Risers match what the walker trains on (rl/terrain.py: up to 5 cm).
STAIRS = [("stairs_north", 0.8, 3.0, "x", 4, 0.045, 0.32, 0.6, 0.9),
          ("stairs_east", 3.35, -1.05, "y", 3, 0.03, 0.30, 0.5, 1.1)]
PLATFORMS = [("platform_sw", -3.1, -3.1, 0.9, 0.9, 0.08)]   # (name, x, y, size x, size y, height): a step-up


def stair_boxes():
    """Each staircase as nested boxes: step i is a box spanning everything at or above its height."""
    out = []
    for name, x, y, axis, n, rise, tread, landing, width in STAIRS:
        for i in range(1, n + 1):
            length = landing + 2 * (n - i + 1) * tread
            sx, sy = (length, width) if axis == "x" else (width, length)
            out.append((f"{name}_{i}", x, y, sx, sy, i * rise, 0, (0.55, 0.55, 0.6)))
    for name, x, y, sx, sy, h in PLATFORMS:
        out.append((name, x, y, sx, sy, h, 0, (0.5, 0.5, 0.56)))
    return out


def obstacles_wbt():
    out = []
    for name, x, y, sx, sy, h, yaw, (r, g, b) in OBSTACLES + stair_boxes():
        out.append(f'Solid {{ translation {x} {y} {h / 2} rotation 0 0 1 {math.radians(yaw):.4f} '
                   f'children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.9 metalness 0 }} '
                   f'geometry Box {{ size {sx} {sy} {h} }} }} ] name "{name}" boundingObject Box {{ size {sx} {sy} {h} }} }}')
    return "\n".join(out) + "\n"


def body_cam_hfov():
    """Horizontal field of view (deg) of the navigator's camera model (rl/camera.py)."""
    sys.path.insert(0, str(HERE.parents[1] / "rl"))
    import camera
    return camera.HFOV_DEG


def wb_shapes(items):
    return " ".join(f'Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.85 metalness 0 }} '
                    f'geometry Mesh {{ url [ "../meshes/{n}.stl" ] }} }}' for n, (r, g, b) in items)


def wb_bounding(cols):
    out = []
    for kind, dims, o, *_ in cols:
        if kind == "mesh":
            g, rot = f'Mesh {{ url [ "../meshes/{dims}.stl" ] }}', "0 0 1 0"
        elif kind == "box":
            g, rot = f"Box {{ size {f3(dims)} }}", "0 0 1 0"
        elif kind == "cyl":
            g, rot = f"Cylinder {{ radius {dims[0]:.5f} height {dims[1]:.5f} }}", "0 1 0 1.5708"
        else:
            g, rot = f"Sphere {{ radius {dims[0]:.5f} }}", "0 0 1 0"
        out.append(f"Pose {{ translation {f3(o)} rotation {rot} children [ {g} ] }}")
    return "Group { children [ " + " ".join(out) + " ] }"


def with_armature(props, axis):
    """Add the servo's rotor inertia (spec.ARMATURE) about the joint axis to the child link.

    MuJoCo models it as joint armature; Webots has no equivalent. Without it the light knee link
    is so light that the servo's damping, applied once per physics step, becomes unstable and the
    legs chatter. Pure rotational inertia doesn't depend on where the axis sits, so adding it to
    the link's inertia about its centre of mass is equivalent.
    """
    m, c, I = props
    I = [row[:] for row in I]
    k = {"0 0 1": 2, "0 1 0": 1, "1 0 0": 0}[axis]
    I[k][k] += spec.ARMATURE
    return m, c, I


def wb_physics(p):
    m, c, I = p
    return (f"Physics {{ density -1 mass {m:.4f} centerOfMass [ {f3(c)} ] "
            f"inertiaMatrix [ {I[0][0]:.3e} {I[1][1]:.3e} {I[2][2]:.3e} {I[0][1]:.3e} {I[0][2]:.3e} {I[1][2]:.3e} ] }}")


def write_proto(body_parts, leg_parts, yt):
    cols = leg_collisions(yt)

    def visuals(lk):
        return wb_shapes([(n, COLORS[c]) for n, (l, _, c, _) in leg_parts.items() if l == lk])

    # The robot spawns in the standing pose: each child frame sits on its joint axis, so only
    # its rotation changes with the joint angle.
    def hinge(i, jn, anchor, axis, lk, inner, q):
        a = f3(v / 1000 for v in anchor)
        return (f'HingeJoint {{ jointParameters HingeJointParameters {{ position {q} anchor {a} axis {axis} minStop -1.5708 maxStop 1.5708 '
                f'dampingConstant {spec.JOINT_DAMPING} staticFriction {spec.JOINT_FRICTION} }} '
                f'device [ RotationalMotor {{ name "{jn}_leg{i}" minPosition -1.5708 maxPosition 1.5708 maxTorque {spec.STALL_TORQUE} maxVelocity {spec.SERVO_FREE_SPEED} }} '
                f'PositionSensor {{ name "{jn}_leg{i}_sensor" }} ] '
                f'endPoint Solid {{ translation {a} rotation {axis} {q} children [ {visuals(lk)} {inner} ] name "leg{i}_{URDF_LINK[lk]}" '
                f'boundingObject {wb_bounding(cols[lk])} physics {wb_physics(with_armature(leg_link_props(leg_parts, lk), axis))} }} }}')

    legs = []
    for i, deg in body.LEG_ANGLES.items():
        foot = (f'TouchSensor {{ name "leg{i}_foot" translation {leg.L3 / 1000:.5f} {yt / 1000:.5f} 0 '
                f'boundingObject Sphere {{ radius {leg.FOOT_R / 1000 + 0.001:.4f} }} physics Physics {{ mass 0.001 }} }}')
        q1, q3, q4 = STANDING
        chain = hinge(i, "joint1", (0, 0, 0), "0 0 1", "coxa",
                      hinge(i, "joint3", (leg.L1, 0, 0), "0 1 0", "femur",
                            hinge(i, "joint4", (leg.L2, 0, 0), "0 1 0", "tibia", foot, q4), q3), q1)
        hip = body.hip_location(i).toTuple()[0]
        # The mount needs its own Physics: Webots anchors a joint whose parent Solid has no physics
        # to the world, which leaves the legs standing while the body falls away. With physics and
        # no joint to the Robot, Webots fuses the mount rigidly onto the body.
        legs.append(f'Solid {{ translation {f3(v / 1000 for v in hip)} rotation 0 0 1 {math.radians(deg):.6f} '
                    f'children [ {visuals("mount")} {chain} ] name "leg{i}_mount" boundingObject {wb_bounding(cols["mount"])} '
                    f'physics {wb_physics(leg_link_props(leg_parts, "mount"))} }}')

    items = [(s, body.PRINT_DENSITY if k == "printed" else None, None if k == "printed" else body.ELECTRONICS_MASS[n])
             for n, (s, _, k) in body_parts.items()]

    text = f"""#VRML_SIM R2025a utf8
# Generated by build.py - Rocky-Bob: rock body + four legs ({spec.SERVO_NAME} servos). Motor names match Bob's controller.
PROTO RockyBob [
  field SFVec3f    translation 0 0 0
  field SFRotation rotation    0 0 1 0
  field SFString   name        "Bob"
  field SFString   controller  "<extern>"
] {{
  Robot {{
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    supervisor TRUE   # lets the bridge read the target marker's position for the camera model
    selfCollision TRUE   # legs collide with the body and each other: only the servos and the shape limit them
    children [
      InertialUnit {{ name "imu" translation 0 0 0.05 }}
      Camera {{ name "camera" translation {body.CAM_D / 1000:.4f} 0 {body.CAM_Z / 1000:.4f} fieldOfView {math.radians(body_cam_hfov()):.4f} width 320 height 200 }}
      Lidar {{ name "lidar" translation 0 0 {body.LIDAR_SCAN_Z / 1000:.4f} horizontalResolution 500 fieldOfView 6.2832 numberOfLayers 1 verticalFieldOfView 0.05 minRange 0.05 maxRange 12 noise 0.01 }}   # RPLIDAR C1
      RangeFinder {{ name "tof" translation {f3(v / 1000 for v in body.tof_point()[0].toTuple())} rotation 0 1 0 {math.radians(body.TOF_TILT):.4f} width 8 height 8 fieldOfView 0.7854 minRange 0.02 maxRange 4 }}   # floor depth sensor (VL53L5CX)
      Gyro {{ name "gyro" translation 0 0 0.05 }}
      GPS {{ name "gps" }}
      {wb_shapes([(n, COLORS[c]) for n, (_, c, _) in body_parts.items()])}
      {chr(10).join("      " + l for l in legs)}
    ]
    boundingObject {wb_bounding(body_collision(body_parts))}
    physics {wb_physics(props(items))}
  }}
}}
"""
    (HERE / "sim/protos").mkdir(parents=True, exist_ok=True)
    (HERE / "sim/protos/RockyBob.proto").write_text(text)
    (HERE / "sim/worlds").mkdir(exist_ok=True)
    world = """#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "../protos/RockyBob.proto"
WorldInfo {
  basicTimeStep 4
  contactProperties [ ContactProperties { coulombFriction [ %FRICTION% ] } ]
}
Viewpoint { orientation -0.25 0.2 0.95 1.95 position 0.7 -1.9 1.3 follow "Bob" followType "Tracking Shot" }
TexturedBackground {}
TexturedBackgroundLight {}
RectangleArena { floorSize 8 8 wallHeight 0.6 }
%OBSTACLES%RockyBob { translation 0 0 0.19 }
# Camera target: an ArUco-style 15 cm marker on a post (rl/camera.py MARKER_SIZE). Drag it around; with BOB_NAV the robot walks to it.
DEF TARGET Solid {
  translation 1 0.4 0
  children [
    Pose { translation 0 0 0.125 children [ Shape { appearance PBRAppearance { baseColor 0.9 0.4 0.1 roughness 0.6 } geometry Cylinder { height 0.25 radius 0.012 } } ] }
    Pose { translation 0 0 0.25 children [ Shape { appearance PBRAppearance { baseColor 0.05 0.05 0.05 roughness 0.8 } geometry Box { size 0.01 0.15 0.15 } } ] }
    Pose { translation 0 0 0.25 children [ Shape { appearance PBRAppearance { baseColor 1 1 1 roughness 0.8 } geometry Box { size 0.012 0.07 0.07 } } ] }
  ]
  name "target"
}
""".replace("%FRICTION%", str(spec.FOOT_FRICTION))
    (HERE / "sim/worlds/rocky_bob.wbt").write_text(world.replace("%OBSTACLES%", obstacles_wbt()))
    # the same arena with nothing in it, for speed tests (rl/webots_eval.py)
    (HERE / "sim/worlds/rocky_bob_open.wbt").write_text(world.replace("%OBSTACLES%", ""))


# ================================================================ main
def main():
    assert abs(lidar.LIDAR_Z - body.LIDAR_SCAN_Z / 1000) < 1e-6, "update rl/lidar.py LIDAR_Z to body.LIDAR_SCAN_Z"
    body_parts = body.build()
    leg_parts, yt = leg.build()   # each part is already in its own link frame
    for d in ("step", "stl", "sim/meshes"):
        (HERE / d).mkdir(parents=True, exist_ok=True)

    for name, (shape, _, kind) in body_parts.items():
        if kind == "printed":
            cq.exporters.export(shape, str(HERE / f"step/{name}.step"))
            cq.exporters.export(shape, str(HERE / f"stl/{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(shape.scale(0.001), str(HERE / f"sim/meshes/{name}.stl"),
                            tolerance=0.0002, angularTolerance=0.3)
    for name, (lk, shape, _, printed) in leg_parts.items():
        if printed:
            cq.exporters.export(shape, str(HERE / f"step/{name}.step"))
            cq.exporters.export(shape, str(HERE / f"stl/{name}.stl"), tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(shape.scale(0.001), str(HERE / f"sim/meshes/{name}.stl"),
                            tolerance=0.0002, angularTolerance=0.3)

    assembly(body_parts, leg_parts, (0, 0, 0)).export(str(HERE / "step/rocky_bob_assembly.step"))
    assembly(body_parts, leg_parts, STANDING).export(str(HERE / "step/rocky_bob_standing.step"))

    # the rock's convex shell, without lumps, as the body's collision shape (body_collision)
    cq.exporters.export(body.shell_at(0).scale(0.001), str(HERE / "sim/meshes/rock_collision.stl"),
                        tolerance=0.0005, angularTolerance=0.5)
    write_urdf(body_parts, leg_parts, yt)
    write_proto(body_parts, leg_parts, yt)

    m, c, _ = body_props(body_parts)
    total = m + 4 * sum(leg_link_props(leg_parts, lk)[0] for lk in LEG_LINKS)
    bb = body_parts["rock_base"][0].BoundingBox()
    print(f"body {m * 1000:.0f} g, robot {total * 1000:.0f} g, "
          f"base footprint {bb.xmax - bb.xmin:.0f} x {bb.ymax - bb.ymin:.0f} mm")


if __name__ == "__main__":
    main()
