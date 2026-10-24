"""Rock body for Bob, styled after Rocky from Project Hail Mary.

A hollow, low-poly boulder with a flat base. The top lifts off like a crab's carapace:
a lip on the base locates it and four 6x3 mm magnets hold it down. Inside, a divider
splits the cavity: the front half (+X) carries a Raspberry Pi 5 and a Pimoroni Servo 2040,
the back half (-X) is a power-supply bay with strap loops and adjustable corner guides.

Body frame: origin at the centre of the flat base, +X forward, +Z up, units mm.
The rock is the intersection of half-spaces (n.p <= d), so it is convex: every leg sits
outside its own flat mounting facet and can never swing into the shell.
"""

import math
import random

import cadquery as cq
import numpy as np

import leg
from leg import box, convex, cut, cyl, fib_dirs, fuse, pebble

# ---------------------------------------------------------------- overall rock
RX, RY, RZ = 124.0, 104.0, 110.0   # semi-axes of the boulder before faceting
RZ_CENTER = 20.0                    # ellipsoid centre height; base is cut flat at z=0
FACETS = 64                         # more = smoother rock
ROUGHNESS = 7.0                     # mm of random facet offset
SEED = 7
WALL = 3.0

# ---------------------------------------------------------------- legs
Z_HIP = 60.0                        # hip-yaw axis height above the base
PAD_D = 106.0                       # distance of each flat leg pad from the centre axis
# Leg index -> outward direction (deg). Order matches Bob's existing leg0..leg3.
LEG_ANGLES = {0: 225.0, 1: 135.0, 2: 45.0, 3: -45.0}
PAD_BACKING = 4.0                   # extra thickness behind each pad for the nut traps

# ---------------------------------------------------------------- lid
Z_SPLIT = 88.0
LIP_H, LIP_T, LIP_GAP = 6.0, 1.8, 0.25
MAGNET_D, MAGNET_H = 6.0, 3.0
POST_R = 5.5
LUMP_SEEDS = (101, 102, 103, 104, 105)   # five carapace lumps round the lidar plinth

# ---------------------------------------------------------------- lidar (top of the lid)
# Slamtec RPLIDAR C1: 55.6 x 55.6 x 41.3 mm, 110 g, 4x M2.5 threaded holes in its base
# (screws may go at most 4 mm in). It sits in a shallow square pocket on a hollow plinth
# in the middle of the lid; the cable drops through a slot into the cavity, to a USB
# adapter on the Pi. The hole spacing isn't in the datasheet text: list it in LIDAR_HOLES
# once measured, or drill the 3 mm plate using the lidar as a template (M2.5 x 6 from below).
LIDAR_SIZE, LIDAR_H, LIDAR_MASS = 55.6, 41.3, 0.110
LIDAR_TOP = 138.0                   # plinth top (the lidar's base) above the rock's base, mm
LIDAR_POCKET = 0.4                  # clearance round the lidar in the pocket
LIDAR_POCKET_D = 2.0
LIDAR_PLATE = 3.0                   # plinth top thickness under the pocket
LIDAR_PLINTH = 64.0                 # plinth width at the top; it flares 4 mm down to the lid
LIDAR_HOLES = []                    # (x, y) of the C1's M2.5 holes relative to its centre: VERIFY on the part
LIDAR_SCAN_Z = LIDAR_TOP + 28.0     # approx. height of the laser's scan plane (VERIFY)
LIDAR_CABLE_SLOT = (16.0, 8.0)      # x, y of the cable slot, at the back of the pocket

# ---------------------------------------------------------------- floor depth sensor (bottom of the nose)
# SparkFun Qwiic ToF Imager, VL53L5CX (SEN-18642; amazon.com/dp/B09M97318D): 8x8 zones, 63 deg diagonal
# field of view (~45 x 45 deg), up to 4 m. The 2D lidar only sees a slice ~36 cm up, so steps, kerbs and
# ledges are invisible to it; this looks forward-down at the floor 0.25-2 m ahead instead. It sits behind
# a small window in a chamfer along the bottom of the nose, tilted TOF_TILT down, below the mustache.
# The board is 1 x 1 in; its 4 corner holes are modelled at SparkFun's usual 0.8 in square: VERIFY.
TOF_TILT = 24.0                     # deg below horizontal the sensor looks
TOF_CENTRE = (103.5, 16.0)          # (x, z) the chamfer plane passes through, mm
TOF_Z = 13.0                        # sensor height above the rock's base, on the chamfer, mm
TOF_BOARD = 25.4                    # mm square
TOF_HOLES = [(sx * 10.16, sy * 10.16) for sx in (-1, 1) for sy in (-1, 1)]   # board frame, mm (VERIFY)
TOF_WINDOW = 6.0                    # window radius: clears the 63 deg cone through the 3 mm wall
TOF_STANDOFF = 3.0
TOF_MASS = 0.003


def tof_normal():
    t = math.radians(TOF_TILT)
    return cq.Vector(math.cos(t), 0, -math.sin(t))


def tof_point():
    """Sensor position (mm, body frame) on the outside of the chamfer, and its unit view direction."""
    n = tof_normal()
    d = n.x * TOF_CENTRE[0] + n.z * TOF_CENTRE[1]
    x = (d - n.z * TOF_Z) / n.x
    return cq.Vector(x, 0, TOF_Z), n


# ---------------------------------------------------------------- mustache
# The lidar looks like a top hat, so Bob gets a handlebar mustache to match: a 3 mm printed
# shell that follows the rock's facets just under the camera "nose". Glue it on (print it in
# dark brown/black, face down).
STACHE_Z = 55.0                     # height of its centre on the nose
STACHE_T = 3.0                      # thickness, standing proud of the rock
STACHE_SCALE = 1.0                  # 1 = about 92 mm wide
# right half: centreline (y, z) from the middle out to the curled tip, and its half-thickness there
STACHE_LINE = [(0, 0), (8, 1.0), (16, 0.6), (24, -0.8), (31, -2.2), (37, -1.6), (41.5, 1.2), (43.5, 5.5),
               (42, 9.5), (38.5, 10.0), (37.5, 7.5)]
STACHE_R = [4.0, 5.2, 5.0, 4.2, 3.4, 2.8, 2.3, 1.9, 1.6, 1.4, 1.2]

# ---------------------------------------------------------------- electronics (front half)
M25_PILOT = 2.2                     # both boards use M2.5 holes; M3 won't fit them
STANDOFF_H, STANDOFF_R = 6.0, 3.0
# Raspberry Pi 5: 85 x 56 board, holes 3.5 mm in, 58 x 49 pattern. Long side along Y,
# ports face -Y.
PI_X0, PI_Y0 = 8.0, -68.0
PI_W, PI_L = 56.0, 85.0
PI_HOLES = [(PI_X0 + x, PI_Y0 + PI_L - y) for x in (3.5, 52.5) for y in (3.5, 61.5)]
PI_HEIGHT = 18.0                    # tallest component above the PCB (USB/Ethernet)
# Pimoroni Servo 2040: 62 x 42, M2.5 holes 2.7 mm in from each edge.
S2040_X0, S2040_Y0 = 8.0, 24.0
S2040_W, S2040_L = 62.0, 42.0
S2040_HOLES = [(S2040_X0 + x, S2040_Y0 + y) for x in (2.7, 59.3) for y in (2.7, 39.3)]
S2040_HEIGHT = 10.0
PCB_T = 1.6

# ---------------------------------------------------------------- power bay (back half)
# Placeholder until the real supply is chosen: set these and re-run build.py.
PSU_L, PSU_W, PSU_H = 80.0, 60.0, 40.0   # along X, Y, Z
PSU_CX = -55.0
PSU_MASS = 0.25                          # kg, for the simulation
GUIDE_H, GUIDE_T, GUIDE_LEN = 8.0, 2.0, 10.0
STRAP_W = 20.0                           # velcro strap width
SWITCH_D, SWITCH_Z = 12.0, 55.0          # panel-mount switch through the back wall

# ---------------------------------------------------------------- Pi camera (front)
# Raspberry Pi Camera Module 3 behind a flat window on the nose of the rock, looking forward.
# Board 25 x 24 mm, M2 holes on 21 x 12.5 mm; lens ~6.5 mm proud of the board. Hole and lens
# positions are from the Module 3 drawing: check them against your board. rl/camera.py has the
# matching lens position (CAMERA_X) and field of view for the navigator.
CAM_D = 108.0                            # window plane distance from the centre (+X)
CAM_Z = 70.0                             # lens height above the base
CAM_LENS_H = 6.5                         # lens height above the board
CAM_APERTURE = 7.0                       # lens hole radius
CAM_BOARD = (25.0, 24.0)                 # width (y), height (z)
CAM_LENS_FROM_TOP = 9.5                  # lens centre below the board's top edge
CAM_HOLES = [(y, z) for y in (-10.5, 10.5) for z in (7.5, -5.0)]   # relative to the lens centre
M2_PILOT = 1.6

PRINT_DENSITY = leg.PRINT_DENSITY
ELECTRONICS_MASS = {"pi5": 0.046, "servo2040": 0.015, "psu": PSU_MASS, "pi_camera": 0.004,
                    "lidar": LIDAR_MASS, "tof": TOF_MASS}


# ================================================================ geometry
def unit(deg):
    a = math.radians(deg)
    return cq.Vector(math.cos(a), math.sin(a), 0)


def hip_location(i):
    """Leg frame -> body frame for leg i."""
    u = unit(LEG_ANGLES[i])
    p = u * (PAD_D - leg.HIP_PLATE_X)
    p.z = Z_HIP
    return cq.Location(p, cq.Vector(0, 0, 1), LEG_ANGLES[i])


def leg_point(i, p):
    t = (hip_location(i) * cq.Location(cq.Vector(*p))).toTuple()[0]
    return cq.Vector(*t)


def _planes():
    """(normal, d) for the rock's random facets, leg pads and base."""
    rnd = random.Random(SEED)
    centre = cq.Vector(0, 0, RZ_CENTER)
    rock = []
    for u in fib_dirs(FACETS, zmin=-0.3):
        u = cq.Vector(*u)
        h = math.sqrt((RX * u.x) ** 2 + (RY * u.y) ** 2 + (RZ * u.z) ** 2)   # ellipsoid support
        rock.append([u, u.dot(centre) + h + rnd.uniform(-ROUGHNESS, ROUGHNESS)])
    tn = tof_normal()
    pads = [(unit(a), PAD_D) for a in LEG_ANGLES.values()] + [(cq.Vector(1, 0, 0), CAM_D),
                                                               (tn, tn.x * TOF_CENTRE[0] + tn.z * TOF_CENTRE[1])]
    # Push any facet that would clip a pad or the camera window outward so each stays a full flat face.
    corners = [leg_point(i, (leg.HIP_PLATE_X, y, z))
               for i in LEG_ANGLES for y in (-28, 28) for z in (-44, 24)]
    corners += [cq.Vector(CAM_D, y, z) for y in (-18, 18) for z in (CAM_Z - 16, CAM_Z + 16)]
    tc, _ = tof_point()
    along = cq.Vector(math.sin(math.radians(TOF_TILT)), 0, math.cos(math.radians(TOF_TILT)))   # up the chamfer
    corners += [tc + cq.Vector(0, y, 0) + along * a for y in (-17, 17) for a in (-8, 14)]
    for f in rock:
        f[1] = max(f[1], max(f[0].dot(c) for c in corners) + 2)
    return [tuple(f) for f in rock], pads


ROCK_PLANES, PAD_PLANES = _planes()
BASE_PLANE = (cq.Vector(0, 0, -1), 0.0)


def shell_at(k, floor=None):
    """The rock shrunk inward by k (floor raised by `floor`, default k)."""
    planes = [(n, d - k) for n, d in ROCK_PLANES + PAD_PLANES]
    planes.append((BASE_PLANE[0], -(k if floor is None else floor)))
    return convex(planes, bound=400)


def slab(z0, z1):
    return box(-400, 400, -400, 400, z0, z1)


def inner_radius(k, deg, z):
    """Distance from the Z axis to the surface of shell_at(k) along direction deg at height z."""
    u = unit(deg)
    best = 1e9
    for n, d in ROCK_PLANES + PAD_PLANES:
        h = n.x * u.x + n.y * u.y
        if h > 1e-6:
            best = min(best, (d - k - n.z * z) / h)
    return best


def top_height(x, y):
    """Height of the outer rock surface above (x, y)."""
    best = 1e9
    for n, d in ROCK_PLANES:
        if n.z > 1e-3:
            best = min(best, (d - n.x * x - n.y * y) / n.z)
    return best


def hex_x(af, x0, x1, y, z):
    return (cq.Workplane("YZ", origin=(x0, y, z))
            .polygon(6, af / math.cos(math.pi / 6)).extrude(x1 - x0).val())


def lumps():
    """Rocky-style carapace bumps: five round the top (the middle is the lidar plinth)."""
    out = []
    spots = [(0.5 * RX * math.cos(math.radians(90 + 72 * k)),
                                   0.5 * RY * math.sin(math.radians(90 + 72 * k)), 24.0)
                                  for k in range(5)]
    keep = cut(slab(Z_SPLIT + 6, 400), *[cq.Solid.makeCylinder(5000, 5000, n * d, n) for n, d in PAD_PLANES])
    for (x, y, r), seed in zip(spots, LUMP_SEEDS):
        z = top_height(x, y) - 0.45 * r
        out.append(pebble((x, y, z), r, seed, n=36, jitter=0.18).intersect(keep))
    return out


def frustum(r0, r1, z0, z1):
    """Square frustum, half-width r0 at z0 tapering to r1 at z1."""
    return (cq.Workplane("XY", origin=(0, 0, z0)).rect(2 * r0, 2 * r0)
            .workplane(offset=z1 - z0).rect(2 * r1, 2 * r1).loft().val())


def tof_board():
    """The SparkFun board (25.4 mm square, 1.6 mm) behind the chamfer window, with the sensor on it."""
    tp, tn = tof_point()
    t = math.radians(TOF_TILT)
    board = box(-0.8, 0.8, -TOF_BOARD / 2, TOF_BOARD / 2, -TOF_BOARD / 2, TOF_BOARD / 2)
    chip = box(0.8, 2.3, -3.2, 3.2, -1.5, 1.5)
    face = tp - tn * (WALL + TOF_STANDOFF + 0.8)
    return fuse(board, chip).rotate((0, 0, 0), (0, 1, 0), math.degrees(t)).translate(face.toTuple())


def mustache():
    """Handlebar mustache conforming to the rock's nose under the camera."""
    line, r = np.array(STACHE_LINE, float), np.array(STACHE_R, float)
    t = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(line, axis=0).T))])
    ts = np.linspace(0, t[-1], int(t[-1] / 0.8))
    ys, zs, rs = (np.interp(ts, t, v) for v in (line[:, 0], line[:, 1], r))
    sk = cq.Sketch()
    for side in (1, -1):
        for y, z, rr in zip(ys, zs, rs):
            sk = sk.push([(side * y * STACHE_SCALE, z * STACHE_SCALE)]).circle(rr * STACHE_SCALE).reset()
    blank = cq.Workplane("YZ", origin=(CAM_D - 30, 0, STACHE_Z)).placeSketch(sk.clean()).extrude(60).val()
    return cut(blank.intersect(shell_at(-STACHE_T)), shell_at(-0.05))


def magnet_spots():
    """(x, y) of the four lid magnets: front, back, left and right, just inside the lip."""
    k = WALL + LIP_GAP + LIP_T
    pts = []
    for deg in (60, 120, 240, 300):   # clear of the camera on the nose and the PSU switch at the back
        r = inner_radius(k, deg, Z_SPLIT + LIP_H) - 2.0
        u = unit(deg)
        pts.append((u.x * r, u.y * r))
    return pts


def build():
    outer = shell_at(0)
    cavity = shell_at(WALL)

    # ---------------------------------------------------------------- base
    base = cut(outer.intersect(slab(-1, Z_SPLIT)), cavity)

    lip = fuse(
        cut(shell_at(WALL - 0.5).intersect(slab(Z_SPLIT - 8, Z_SPLIT)), shell_at(WALL + LIP_GAP + LIP_T)),
        cut(shell_at(WALL + LIP_GAP).intersect(slab(Z_SPLIT, Z_SPLIT + LIP_H)), shell_at(WALL + LIP_GAP + LIP_T)),
    )
    inside = shell_at(WALL - 0.5, floor=WALL - 0.5)   # for clipping internal features into the walls
    lid_clear = shell_at(WALL + LIP_GAP)

    posts, post_pockets = [], []
    for x, y in magnet_spots():
        z_top = Z_SPLIT + LIP_H
        post = fuse(cyl(POST_R, (x, y, Z_SPLIT - 22), (x, y, z_top)),
                    cq.Solid.makeCone(0.5, POST_R, 10, cq.Vector(x, y, Z_SPLIT - 32)))
        posts.append(fuse(post.intersect(slab(-1, Z_SPLIT)).intersect(inside),
                          post.intersect(slab(Z_SPLIT, z_top)).intersect(lid_clear)))
        post_pockets.append(cyl(MAGNET_D / 2 + 0.1, (x, y, z_top - MAGNET_H - 0.2), (x, y, z_top + 1)))

    divider = box(-1.5, 1.5, -400, 400, WALL - 0.5, Z_SPLIT - 4).intersect(inside)
    divider_notch = box(-5, 5, -20, 20, WALL, WALL + 22)

    standoffs, pilots = [], []
    for x, y in PI_HOLES + S2040_HOLES:
        standoffs.append(cyl(STANDOFF_R, (x, y, WALL - 0.5), (x, y, WALL + STANDOFF_H)))
        pilots.append(cyl(M25_PILOT / 2, (x, y, WALL), (x, y, WALL + STANDOFF_H + 1)))

    # power bay: corner guides and two strap loops
    gx, gy = PSU_L / 2 + 0.5, PSU_W / 2 + 0.5
    guides = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = PSU_CX + sx * gx, sy * gy
            guides.append(box(*sorted((cx, cx + sx * GUIDE_T)), *sorted((cy - sy * GUIDE_LEN, cy + sy * GUIDE_T)),
                              WALL - 0.5, WALL + GUIDE_H))
            guides.append(box(*sorted((cx - sx * GUIDE_LEN, cx + sx * GUIDE_T)), *sorted((cy, cy + sy * GUIDE_T)),
                              WALL - 0.5, WALL + GUIDE_H))
    loops, loop_holes = [], []
    for sy in (-1, 1):
        y = sy * (PSU_W / 2 + 6)
        loops.append(box(PSU_CX - STRAP_W / 2 - 3, PSU_CX + STRAP_W / 2 + 3, y - 3, y + 3, WALL - 0.5, WALL + 6))
        loop_holes.append(box(PSU_CX - STRAP_W / 2 - 0.5, PSU_CX + STRAP_W / 2 + 0.5, y - 4, y + 4, WALL, WALL + 3))

    # leg pads: backing, M3 clearance holes with nut traps inside, wire pass-through
    pad_add, pad_cut = [], []
    for i in LEG_ANGLES:
        loc = hip_location(i)
        px = leg.HIP_PLATE_X
        back = box(px - WALL - PAD_BACKING, px, -26, 26, -42, 22)
        pad_add.append(back.moved(loc).intersect(outer))
        for y, z in leg.HIP_HOLES:
            pad_cut.append(cyl(leg.M3_CLEAR / 2, (px - 20, y, z), (px + 1, y, z)).moved(loc))
            pad_cut.append(hex_x(leg.M3_NUT_AF, px - WALL - PAD_BACKING - 0.01,
                                 px - WALL - PAD_BACKING + leg.M3_NUT_H, y, z).moved(loc))
        (y0, y1), (z0, z1) = leg.HIP_WIRE_SLOT
        pad_cut.append(box(px - 20, px + 1, y0, y1, z0, z1).moved(loc))

    switch = cyl(SWITCH_D / 2, (-300, 0, SWITCH_Z), (PSU_CX - PSU_L / 2 - 2, 0, SWITCH_Z))

    # camera: lens hole through the window, M2 standoffs from the inside of the window to the board
    inner = CAM_D - WALL
    board_x = inner - CAM_LENS_H
    camera_cut = [cyl(CAM_APERTURE, (inner - 1, 0, CAM_Z), (CAM_D + 1, 0, CAM_Z))]
    camera_add = []
    for y, z in CAM_HOLES:
        camera_add.append(cyl(2.25, (board_x, y, CAM_Z + z), (inner + 0.5, y, CAM_Z + z)))
        camera_cut.append(cyl(M2_PILOT / 2, (board_x - 1, y, CAM_Z + z), (inner - 0.5, y, CAM_Z + z)))

    # floor depth sensor: a window through the chamfer, the board on four standoffs behind it
    tp, tn = tof_point()
    up = cq.Vector(math.sin(math.radians(TOF_TILT)), 0, math.cos(math.radians(TOF_TILT)))
    side = cq.Vector(0, 1, 0)
    inner_tof = tp - tn * WALL
    tof_cut = [cyl(TOF_WINDOW, (tp + tn * 1).toTuple(), (inner_tof - tn * 1).toTuple())]
    tof_add = []
    for a, b in TOF_HOLES:
        c = inner_tof + side * a + up * b
        tof_add.append(cyl(2.25, (c + tn * 0.5).toTuple(), (c - tn * TOF_STANDOFF).toTuple()))
        tof_cut.append(cyl(M25_PILOT / 2, (c + tn * 0.2).toTuple(), (c - tn * (TOF_STANDOFF + 1)).toTuple()))

    base = fuse(base, lip, *posts, divider, *standoffs, *guides, *loops, *pad_add, *camera_add, *tof_add)
    base = cut(base, *post_pockets, divider_notch, *pilots, *loop_holes, *pad_cut, switch, *camera_cut, *tof_cut)

    # ---------------------------------------------------------------- lid
    lid = fuse(outer, *lumps()).intersect(slab(Z_SPLIT, 400))
    lid = cut(lid, cavity)
    bosses, boss_pockets = [], []
    for x, y in magnet_spots():
        z0 = Z_SPLIT + LIP_H + 0.4
        bosses.append(cyl(POST_R, (x, y, z0), (x, y, 300)).intersect(outer))
        boss_pockets.append(cyl(MAGNET_D / 2 + 0.1, (x, y, z0 - 1), (x, y, z0 + MAGNET_H + 0.2)))
    lid = cut(fuse(lid, *bosses), *boss_pockets)

    # lidar plinth: a hollow, slightly flared block standing on the lid, pocket on top
    a, b = LIDAR_PLINTH / 2, LIDAR_PLINTH / 2 + 4
    z0 = Z_SPLIT + 20
    plinth = frustum(b, a, z0, LIDAR_TOP)
    hollow = frustum(b - WALL, a - WALL, z0 - 1, LIDAR_TOP - LIDAR_POCKET_D - LIDAR_PLATE)
    q = LIDAR_SIZE / 2 + LIDAR_POCKET
    cx, cy = LIDAR_CABLE_SLOT
    lidar_cut = [hollow,
                 box(-q, q, -q, q, LIDAR_TOP - LIDAR_POCKET_D, LIDAR_TOP + 1),
                 box(-q + 2, -q + 2 + cx, -cy / 2, cy / 2, z0 - 1, LIDAR_TOP + 1)]
    for x, y in LIDAR_HOLES:
        lidar_cut.append(cyl(1.4, (x, y, z0), (x, y, LIDAR_TOP + 1)))
    lid = cut(fuse(lid, plinth), cavity, *lidar_cut)

    # ---------------------------------------------------------------- placeholders (not printed)
    pcb_z = WALL + STANDOFF_H
    pi5 = fuse(box(PI_X0, PI_X0 + PI_W, PI_Y0, PI_Y0 + PI_L, pcb_z, pcb_z + PCB_T),
               box(PI_X0 + 2, PI_X0 + PI_W - 2, PI_Y0 - 2, PI_Y0 + 60, pcb_z + PCB_T, pcb_z + PCB_T + PI_HEIGHT))
    s2040 = fuse(box(S2040_X0, S2040_X0 + S2040_W, S2040_Y0, S2040_Y0 + S2040_L, pcb_z, pcb_z + PCB_T),
                 box(S2040_X0 + 6, S2040_X0 + S2040_W - 6, S2040_Y0 + 6, S2040_Y0 + S2040_L - 6,
                     pcb_z + PCB_T, pcb_z + PCB_T + S2040_HEIGHT))
    psu = box(PSU_CX - PSU_L / 2, PSU_CX + PSU_L / 2, -PSU_W / 2, PSU_W / 2, WALL, WALL + PSU_H)

    return {
        # name: (shape, colour, kind) kind: printed | electronics
        "rock_base": (base, "stone", "printed"),
        "rock_lid": (lid, "stone_light", "printed"),
        "pi5": (pi5, "pcb_green", "electronics"),
        "servo2040": (s2040, "pcb_black", "electronics"),
        "psu": (psu, "psu", "electronics"),
        "pi_camera": (fuse(box(board_x - 1, board_x, -CAM_BOARD[0] / 2, CAM_BOARD[0] / 2,
                               CAM_Z + CAM_LENS_FROM_TOP - CAM_BOARD[1], CAM_Z + CAM_LENS_FROM_TOP),
                           box(board_x, inner - 0.2, -4, 4, CAM_Z - 4, CAM_Z + 4)), "pcb_green", "electronics"),
        "mustache": (mustache(), "mustache", "printed"),
        "tof": (tof_board(), "pcb_red", "electronics"),
        "lidar": (fuse(box(-LIDAR_SIZE / 2, LIDAR_SIZE / 2, -LIDAR_SIZE / 2, LIDAR_SIZE / 2,
                           LIDAR_TOP - LIDAR_POCKET_D, LIDAR_TOP - LIDAR_POCKET_D + 16),
                       cyl(25.0, (0, 0, LIDAR_TOP - LIDAR_POCKET_D + 16), (0, 0, LIDAR_TOP - LIDAR_POCKET_D + LIDAR_H))),
                  "pcb_black", "electronics"),
    }


COLORS = {
    "mustache": (0.10, 0.07, 0.05),
    "pcb_red": (0.75, 0.12, 0.10),
    "pcb_green": (0.10, 0.45, 0.20),
    "pcb_black": (0.08, 0.08, 0.10),
    "psu": (0.25, 0.35, 0.60),
}
