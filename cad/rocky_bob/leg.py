"""Rock-styled 3-DOF leg for Bob (standard-size 25 kg servos, M3 hardware).

Leg frame (zero pose): origin on the hip-yaw axis, +X points out along the leg, +Z up.
  joint1 (hip yaw)   axis Z at (0, 0, 0)
  joint3 (hip pitch) axis Y at (L1, 0, 0)       L1/L2 match Bob's current 107.2 mm spacing
  joint4 (knee)      axis Y at L2 further along X
  foot centre        L3 further along X
The hip pod's mounting plate faces -X; its back face is the plane x = HIP_PLATE_X.
"""

import math
import random

import cadquery as cq
from OCP.gp import gp_Trsf

# ---------------------------------------------------------------- kinematics (mm)
L1 = 107.2   # hip yaw -> hip pitch
L2 = 107.2   # hip pitch -> knee
L3 = 160.0   # knee -> foot centre

# ---------------------------------------------------------------- servo (servo frame)
# Servo frame: output shaft on +Z through the origin. The case runs from x=-SHAFT_FROM_END
# to x=CASE_L-SHAFT_FROM_END. HobbyPark 25 kg coreless, standard size (40 x 20 mm, 49.5 x 10 mm
# hole pattern, 25T spline) - seller drawings vary, so check with calipers. (MG995 was 40.7 x 19.7,
# tabs 54.6, holes 48.7 x 10.)
CASE_L, CASE_W, CASE_H = 40.0, 20.0, 37.0
SHAFT_FROM_END = 10.0
TAB_L, TAB_T = 54.5, 2.5
TAB_TOP_BELOW_CASE_TOP = 10.0
TAB_HOLE_DX, TAB_HOLE_DY = 49.5, 10.0
HORN_FACE_ABOVE_CASE = 8.0   # case top -> outer face of the 25T round horn
HORN_D, HORN_T = 24.0, 2.0
HORN_BCD = 14.0              # bolt circle of the horn's M3 holes - measure yours
SERVO_MASS = 0.062           # kg

# Joint stack along the servo axis: horn face at +G_HORN, idler boss face at -G_IDLER.
G_HORN = 23.5
CASE_TOP = G_HORN - HORN_FACE_ABOVE_CASE          # 15.5
CASE_BOT = CASE_TOP - CASE_H                      # -21.0
FLOOR_T = 5.0
G_IDLER = -CASE_BOT + FLOOR_T                     # 26.0
GAP = 0.5
CHEEK_T = 5.0
CHEEK_R = 16.0
TAB_TOP = CASE_TOP - TAB_TOP_BELOW_CASE_TOP       # 5.5
TAB_BOT = TAB_TOP - TAB_T                         # 3.0
TAB_CX = CASE_L / 2 - SHAFT_FROM_END              # 10.35

# ---------------------------------------------------------------- M3 hardware
M3_CLEAR = 3.4
M3_PILOT = 2.6        # M3 screw self-taps into PLA/PETG
M3_HEAD_D = 6.0
M3_NUT_AF = 5.7       # 5.5 mm nut + clearance
M3_NUT_H = 2.6
FIT = 0.3             # pocket clearance around the servo
WALL = 3.0

# ---------------------------------------------------------------- look
COLORS = {
    "stone": (0.47, 0.42, 0.37),
    "stone_dark": (0.34, 0.31, 0.28),
    "stone_light": (0.60, 0.55, 0.48),
    "servo": (0.10, 0.10, 0.12),
    "horn": (0.85, 0.85, 0.85),
}
FEMUR_R = 24.0
TIBIA_R = 13.0
FOOT_R = 16.0
SPIGOT_R = 7.0
PRINT_DENSITY = 1240 * 0.45   # kg/m^3, PLA at ~45 % effective fill

# Hip pod mounting plate: back face plane, and the 4x M3 pattern on it (leg y, z).
HIP_PLATE_X = -(TAB_CX + TAB_L / 2 + WALL) - 4
HIP_PLATE_Y = (-22, 22)
HIP_PLATE_Z = (-38, 18)
HIP_HOLES = [(y, z) for y in (-15, 15) for z in (-30, 10)]
HIP_WIRE_SLOT = ((2, 20), (-22, -12))   # (y range, z range) where the servo wires leave


# ================================================================ helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cyl(r, p0, p1):
    p0, p1 = cq.Vector(*p0), cq.Vector(*p1)
    return cq.Solid.makeCylinder(r, (p1 - p0).Length, p0, p1 - p0)


def sphere(r, c):
    return cq.Solid.makeSphere(r, cq.Vector(*c), angleDegrees1=-90, angleDegrees2=90)


def hex_prism(af, z0, z1):
    return (cq.Workplane("XY", origin=(0, 0, z0))
            .polygon(6, af / math.cos(math.pi / 6)).extrude(z1 - z0).val())


def fuse(*shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def cut(shape, *tools):
    for t in tools:
        shape = shape.cut(t)
    return shape.clean()


def try_fillet(shape, radius, selector):
    try:
        return cq.Workplane().add(shape).edges(selector).fillet(radius).val()
    except Exception:
        return shape


def frame(cols, t=(0, 0, 0)):
    """Location whose rotation maps servo x/y/z onto the given column vectors."""
    (a, b, c) = cols
    tr = gp_Trsf()
    tr.SetValues(a[0], b[0], c[0], t[0],
                 a[1], b[1], c[1], t[1],
                 a[2], b[2], c[2], t[2])
    return cq.Location(tr)


def place(shape, cols, t=(0, 0, 0)):
    return shape.moved(frame(cols, t))


def halfspace_cut(shape, n, d):
    """Keep the part of `shape` with n.p <= d (n need not be unit)."""
    n = cq.Vector(*n).normalized()
    return shape.cut(cq.Solid.makeCylinder(5000, 5000, n * d, n))


def convex(planes, bound=1000):
    """Convex polyhedron from (normal, d) half-spaces n.p <= d."""
    s = box(-bound, bound, -bound, bound, -bound, bound)
    for n, d in planes:
        s = halfspace_cut(s, n, d)
    return s.clean()


def fib_dirs(n, zmin=-1.0):
    """Roughly even unit directions on the sphere, keeping those with z >= zmin."""
    out = []
    for i in range(n):
        z = 1 - 2 * (i + 0.5) / n
        r = math.sqrt(1 - z * z)
        a = i * math.pi * (3 - math.sqrt(5))
        if z >= zmin:
            out.append((r * math.cos(a), r * math.sin(a), z))
    return out


def pebble(center, r, seed, n=40, jitter=0.12):
    """Low-poly rock: tangent planes of a sphere with random offsets."""
    rnd = random.Random(seed)
    c = cq.Vector(*center)
    planes = []
    for u in fib_dirs(n):
        u = cq.Vector(*u)
        planes.append((u, u.dot(c) + r * (1 + rnd.uniform(-jitter, jitter))))
    return convex(planes, bound=abs(c.x) + abs(c.y) + abs(c.z) + 3 * r)


def facet_ring(x, yc, r, rot_deg, seed, sides=7):
    """Irregular polygon in the plane x = const, used as a loft section."""
    rnd = random.Random(seed)
    pts = []
    for k in range(sides):
        a = math.radians(rot_deg) + 2 * math.pi * k / sides
        rk = r * rnd.uniform(0.92, 1.04)
        pts.append(cq.Vector(x, yc + rk * math.cos(a), rk * math.sin(a)))
    return cq.Wire.makePolygon(pts, close=True)


def faceted_segment(sections, yc, seed):
    """Ruled loft through (x, radius, rotation) sections: a craggy limb segment."""
    wires = [facet_ring(x, yc, r, rot, seed + i) for i, (x, r, rot) in enumerate(sections)]
    return cq.Solid.makeLoft(wires, True)


# Servo orientations. Case always points back toward the parent; child grows along -servo x.
YAW = ((-1, 0, 0), (0, -1, 0), (0, 0, 1))     # shaft up
PITCH = ((-1, 0, 0), (0, 0, 1), (0, 1, 0))    # shaft along +Y


# ================================================================ servo-frame features
def servo_model():
    x0, x1 = -SHAFT_FROM_END, CASE_L - SHAFT_FROM_END
    w = CASE_W / 2
    body = fuse(
        box(x0, x1, -w, w, CASE_BOT, CASE_TOP),
        box(TAB_CX - TAB_L / 2, TAB_CX + TAB_L / 2, -w, w, TAB_BOT, TAB_TOP),
        cyl(6.0, (0, 0, CASE_TOP), (0, 0, CASE_TOP + 2.0)),
        cyl(2.9, (0, 0, CASE_TOP + 2.0), (0, 0, G_HORN - HORN_T)),
    )
    horn = cut(cyl(HORN_D / 2, (0, 0, G_HORN - HORN_T), (0, 0, G_HORN)),
               cyl(2.5, (0, 0, G_HORN - 5), (0, 0, G_HORN + 1)))
    return body, horn


def pocket():
    """Everything a parent link must remove to hold the servo."""
    x0, x1 = -SHAFT_FROM_END - FIT, CASE_L - SHAFT_FROM_END + FIT
    w = CASE_W / 2 + FIT
    top = G_HORN + 20
    tools = [
        box(x0, x1, -w, w, CASE_BOT - FIT, top),
        box(TAB_CX - TAB_L / 2 - FIT, TAB_CX + TAB_L / 2 + FIT, -w, w, TAB_BOT - FIT, top),
        # wire exit low on the far end, out through the side
        box(CASE_L - SHAFT_FROM_END - 8, CASE_L + 20, -45, 0, CASE_BOT, CASE_BOT + 7),
        # idler pivot: M3 through the floor, nut trapped under the servo
        cyl(M3_CLEAR / 2, (0, 0, -G_IDLER - 20), (0, 0, CASE_BOT)),
        hex_prism(M3_NUT_AF, CASE_BOT - M3_NUT_H, CASE_BOT + 0.01),
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = (TAB_CX + sx * TAB_HOLE_DX / 2, sy * TAB_HOLE_DY / 2)
            tools.append(cyl(M3_PILOT / 2, (*p, TAB_BOT - 12), (*p, TAB_BOT + 1)))
    return tools


def housing():
    return box(TAB_CX - TAB_L / 2 - WALL, TAB_CX + TAB_L / 2 + WALL,
               -CASE_W / 2 - WALL, CASE_W / 2 + WALL, -G_IDLER, TAB_TOP)


def clevis(reach):
    """Child-side fork: horn cheek (+z) and idler cheek (-z), reaching `reach` along -x."""
    def cheek(z0, z1):
        return fuse(cyl(CHEEK_R, (0, 0, z0), (0, 0, z1)),
                    box(-reach, 0, -CHEEK_R, CHEEK_R, z0, z1))
    horn_cheek = cheek(G_HORN, G_HORN + CHEEK_T)
    idler_z1 = -G_IDLER - GAP
    idler_cheek = cheek(idler_z1 - CHEEK_T, idler_z1)
    holes = [cyl(4.5, (0, 0, G_HORN - 1), (0, 0, G_HORN + CHEEK_T + 1)),
             cyl(M3_CLEAR / 2, (0, 0, idler_z1 - CHEEK_T - 1), (0, 0, idler_z1 + 1)),
             cyl(M3_HEAD_D / 2 + 0.2, (0, 0, idler_z1 - CHEEK_T - 1), (0, 0, idler_z1 - CHEEK_T + 2.5))]
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        p = (HORN_BCD / 2 * math.cos(a), HORN_BCD / 2 * math.sin(a))
        holes.append(cyl(M3_CLEAR / 2, (*p, G_HORN - 1), (*p, G_HORN + CHEEK_T + 1)))
    return cut(fuse(horn_cheek, idler_cheek), *holes)


def joint_clearance(r):
    """Cylinder between a clevis' cheeks that the child must keep clear of the parent."""
    return cyl(r, (0, 0, -G_IDLER - GAP), (0, 0, G_HORN))


# ================================================================ parts (each in its own link frame)
def build():
    s_body, s_horn = servo_model()

    # --- hip pod (bolts to the rock body): housing for the yaw servo + mounting plate
    plate = box(HIP_PLATE_X, HIP_PLATE_X + 4, *HIP_PLATE_Y, *HIP_PLATE_Z)
    mount_holes = [cyl(M3_CLEAR / 2, (HIP_PLATE_X - 1, y, z), (HIP_PLATE_X + 5, y, z))
                   for y, z in HIP_HOLES]
    hip = fuse(place(housing(), YAW), plate)
    hip = try_fillet(hip, 2.0, "|Z")
    hip = cut(hip, *[place(t, YAW) for t in pocket()], *mount_holes)

    # --- coxa (joint1 link): yaw clevis + web + hip-pitch servo housing.
    # Clevises are fused last so the clearance cut never shares a face with a cheek.
    coxa = fuse(
        box(30, 70, -G_IDLER, 16, -G_IDLER - GAP - CHEEK_T, G_HORN + CHEEK_T),
        place(housing(), PITCH, (L1, 0, 0)),
    )
    coxa = try_fillet(coxa, 3.0, "|Y")
    coxa = fuse(cut(coxa, place(joint_clearance(26), YAW)), place(clevis(70), YAW))
    coxa = cut(coxa, *[place(t, PITCH, (L1, 0, 0)) for t in pocket()])

    # --- femur (joint3 link): craggy segment housing the knee servo
    yc = (-G_IDLER + TAB_TOP) / 2   # segment axis centred on the servo stack
    femur = fuse(
        box(27, 60, -G_IDLER - GAP - CHEEK_T, G_HORN + CHEEK_T, -CHEEK_R, CHEEK_R),
        faceted_segment([(30, 0.75 * FEMUR_R, 0), (55, FEMUR_R, 12), (L2 + 12, FEMUR_R, -8),
                         (L2 + 22, 0.9 * FEMUR_R, 6), (L2 + 32, 0.55 * FEMUR_R, 20)], yc, seed=10),
    )
    femur = fuse(cut(femur, place(joint_clearance(27), PITCH)), place(clevis(60), PITCH))
    femur = cut(femur,
                box(L2 - 25, L2 + 60, -80, -G_IDLER, -40, 40),   # flat for the tibia's idler cheek
                *[place(t, PITCH, (L2, 0, 0)) for t in pocket()])

    # --- tibia (joint4 link): tapering craggy shin ending in a spigot for the foot
    yt = (G_HORN + CHEEK_T - G_IDLER - GAP - CHEEK_T) / 2
    neck = (cq.Workplane("YZ", origin=(62, 0, 0))
            .center(yt, 0).rect(G_HORN + G_IDLER + GAP + 2 * CHEEK_T, 2 * CHEEK_R)
            .workplane(offset=23).circle(TIBIA_R).loft().val())
    tip = L3 - FOOT_R + 2
    tibia = fuse(
        box(36, 62, -G_IDLER - GAP - CHEEK_T, G_HORN + CHEEK_T, -CHEEK_R, CHEEK_R),
        neck,
        faceted_segment([(84, TIBIA_R + 0.5, 0), (L3 - 60, TIBIA_R - 1, 15), (tip, TIBIA_R - 3, -10)],
                        yt, seed=20),
        cyl(SPIGOT_R, (tip - 1, yt, 0), (tip + 8, yt, 0)),
    )
    tibia = fuse(cut(tibia, place(joint_clearance(36), PITCH)), place(clevis(62), PITCH))
    tibia = cut(tibia, cyl(M3_PILOT / 2, (tip - 15, yt, 0), (tip + 9, yt, 0)))

    # --- foot: a pebble (print in TPU), M3 screw up into the tibia spigot
    foot = cut(pebble((L3, yt, 0), FOOT_R, seed=30, n=30),
               cyl(M3_CLEAR / 2, (L3 - FOOT_R - 1, yt, 0), (L3 + FOOT_R + 2, yt, 0)),
               cyl(M3_HEAD_D / 2 + 0.3, (L3 + 2, yt, 0), (L3 + FOOT_R + 2, yt, 0)),
               cyl(SPIGOT_R + 0.3, (tip - 5, yt, 0), (tip + 8.3, yt, 0)),
               tibia)   # seat the pebble on the shin's craggy end

    return {
        # name: (link, shape, colour, is_printed)
        "hip_pod": ("mount", hip, "stone_dark", True),
        "servo_hip_yaw": ("mount", place(s_body, YAW), "servo", False),
        "horn_hip_yaw": ("coxa", place(s_horn, YAW), "horn", False),
        "coxa": ("coxa", coxa, "stone_dark", True),
        "servo_hip_pitch": ("coxa", place(s_body, PITCH, (L1, 0, 0)), "servo", False),
        "horn_hip_pitch": ("femur", place(s_horn, PITCH), "horn", False),
        "femur": ("femur", femur, "stone", True),
        "servo_knee": ("femur", place(s_body, PITCH, (L2, 0, 0)), "servo", False),
        "horn_knee": ("tibia", place(s_horn, PITCH), "horn", False),
        "tibia": ("tibia", tibia, "stone", True),
        "foot": ("tibia", foot, "stone_light", True),
    }, yt

